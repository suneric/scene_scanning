"""
discrete coverage path planning problem
with monte carlo tree search algorithm
"""
import numpy as np
import math
from util import *
from map import *
import sys
import os
import time
import math
import copy
import argparse
import csv
import pandas as pd
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import tensorflow.keras.backend as K
import scipy.signal
import os
from datetime import datetime

import plotly.graph_objects as go
from plotly.subplots import make_subplots

np.random.seed(124)

"""
Environments
"""
class CPPEnv(object):
    def __init__(self,util,coverage):
        self.util = util
        self.vpsState = [0]*len(self.util.viewpoints)
        self.voxelState = [0]*len(self.util.voxels)
        self.occupiedCount = len(np.nonzero(self.voxelState)[0])
        self.vpIdx = 0
        self.targetCoverage = coverage
        self.totalDist = 0.0

    def reset(self,vpIdx=0):
        self.vpIdx = vpIdx
        self.vpsState = [0]*len(self.util.viewpoints)
        self.voxelState = [0]*len(self.util.voxels)
        self.occupiedCount = len(np.nonzero(self.voxelState)[0])
        self.totalDist = 0.0

        vp = self.util.viewpoints[vpIdx]
        self.vpsState[vpIdx] += 1

        # voxel state
        vpCover = [0]*len(self.util.voxels)
        for v in vp.voxels:
            self.voxelState[v] += 1
            vpCover[v] += 1

        coverage = self.coverage()
        nbvps = self.util.neighbors(vpIdx)

        obs = np.zeros((2,len(self.util.voxels)))
        obs[0,:]=self.voxelState
        obs[1,:]=vpCover

        return vp, nbvps, obs, coverage

    def step(self, vpIdx):
        vp = self.util.viewpoints[vpIdx]

        # calculate traveling distance
        vp0 = self.util.viewpoints[self.vpIdx]
        dist = vpDistance(vp0, vp)
        self.totalDist += dist
        self.vpIdx = vpIdx

        # move to new vp and update the status
        self.vpsState[vpIdx] += 1
        vpCover = [0]*len(self.util.voxels)
        for v in vp.voxels:
            self.voxelState[v] += 1
            vpCover[v] += 1

        # calcuate reward
        occupiedCount_new = len(np.nonzero(self.voxelState)[0])
        coverage_add = 100.0*(occupiedCount_new - self.occupiedCount)/len(self.util.voxels)
        self.occupiedCount = occupiedCount_new
        vp_penaulty = -0.1*(self.vpsState[vpIdx]-1) # give penaulty for revisiting a viewpoint
        dist_penaulty = -0.01*dist
        reward = coverage_add + dist_penaulty + vp_penaulty

        coverage = self.coverage()
        nbvps = self.util.neighbors(vpIdx)

        obs = np.zeros((2,len(self.util.voxels)))
        obs[0,:]=self.voxelState
        obs[1,:]=vpCover

        return vp,nbvps,reward,obs,coverage

    def coverage(self):
        return float(self.occupiedCount) / float(len(self.util.voxels))

    def distance(self):
        return self.totalDist

"""
Replay Buffer, strore experiences and calculate total rewards, advanteges
the buffer will be used for update the policy
"""
class ReplayBuffer:
    def __init__(self, input_shape, action_size, size=1000):
        self.obs_buf = np.zeros([size]+list(input_shape), dtype=np.float32) # states
        self.act_buf = np.zeros((size, action_size), dtype=np.float32) # action, based on stochasitc policy with teh probability
        self.rew_buf = np.zeros(size, dtype=np.float32) # step reward
        self.pred_buf = np.zeros((size, action_size), dtype=np.float32) # prediction: action probability, output of actor net
        self.val_buf = np.zeros(size, dtype=np.float32) # value of (s,a), output of critic net
        self.adv_buf = np.zeros(size, dtype=np.float32) # advantege Q(s,a)-V(s)
        self.ret_buf = np.zeros(size, dtype=np.float32) # ep_return, total reward of episode
        self.ptr, self.idx = 0, 0 # buffer ptr, and current trajectory start index

    def store(self, state, action, reward, prediction, value):
        #print("storing", state[0].shape, action.shape, reward, prediction.shape, value.shape)
        self.obs_buf[self.ptr]=state
        self.act_buf[self.ptr]=action
        self.rew_buf[self.ptr]=reward
        self.pred_buf[self.ptr]=prediction
        self.val_buf[self.ptr]=value
        self.ptr += 1

    def size(self):
        return self.ptr

    """
    For each epidode, calculating the total reward and advanteges with specific
    gamma and lamada
    """
    def ep_update(self, gamma=0.99, lamda=0.95):
        """
        magic from rllab for computing discounted cumulative sums of vectors
        input: vector x: [x0, x1, x2]
        output: [x0+discount*x1+discount^2*x2, x1+discount*x2, x2]
        """
        def discount_cumsum(x,discount):
            return scipy.signal.lfilter([1], [1, float(-discount)], x[::-1], axis=0)[::-1]

        ep_slice = slice(self.idx,self.ptr)
        rews = np.append(self.rew_buf[ep_slice],0)
        vals = np.append(self.val_buf[ep_slice],0)
        # rewards-to-go, which is targets for the value function
        self.ret_buf[ep_slice] = discount_cumsum(rews,gamma)[:-1]
        # General Advantege Estimation
        deltas = rews[:-1]+gamma*vals[1:]-vals[:-1]
        self.adv_buf[ep_slice] = discount_cumsum(deltas,gamma*lamda)
        self.idx = self.ptr

    def get(self):
        s = slice(0,self.ptr)
        # normalize advantage batch-wise
        advs = self.adv_buf[s]
        normalized_advs = (advs-np.mean(advs))/(np.std(advs)+1e-10)
        data = dict(states=self.obs_buf[s], actions=self.act_buf[s],
                    returns=self.ret_buf[s], predictions=self.pred_buf[s],
                    advantages=normalized_advs)
        self.ptr, self.idx = 0, 0
        return data


"""
Agent NN
"""
def mlp_net(inputs_dim, outputs_dim, outputs_activation='softmax'):
    inputs = keras.Input(shape=inputs_dim)
    x = layers.Dense(int(inputs_dim[1]/3), activation = 'relu')(inputs)
    x = layers.Dense(64, activation = 'relu')(x)
    x = layers.Flatten()(x)
    outputs = layers.Dense(outputs_dim, activation = outputs_activation)(x)
    return keras.Model(inputs=inputs, outputs=outputs)

"""
loss print call back
"""
class PrintLoss(keras.callbacks.Callback):
    def on_epoch_end(self,epoch,logs={}):
        print("epoch index", epoch+1, "loss", logs.get('loss'))

"""
Actor net
"""
class Actor_Model:
    def __init__(self, input_shape, action_size, clip_ratio, lr, beta):
        self.clip_ratio = clip_ratio
        self.beta = beta # hyperparameter that controls the influence of entropy loss
        self.action_size = action_size
        self.actor = self.build_model(input_shape, action_size, lr)
        self.loss_printer = PrintLoss()

    def build_model(self, input_shape, action_size, lr):
        model = mlp_net(inputs_dim=input_shape, outputs_dim=action_size, outputs_activation="softmax")
        model.compile(loss=self.ppo_loss, optimizer=keras.optimizers.Adam(learning_rate=lr))
        print(model.summary())
        return model

    """
    The key part of the PPO-clip
    policy ratio is define as r = pi(a|s) / pi_old(a|s)
    loss = min(r*AF, clip(r, 1-e, 1+e)*AF), where 'e' is the clip ratio,
    and AF is the advantage function AF(s,a)=Q(s,a)-V(s)
    """
    def ppo_loss(self, y_true, y_pred):
        # y_true: np.hstack([advantages, predictions, actions])
        advs,o_pred,acts = y_true[:,:1],y_true[:,1:1+self.action_size],y_true[:,1+self.action_size:]
        # print(y_pred, advs, picks, acts)
        prob = y_pred*acts
        old_prob = o_pred*acts
        ratio = prob/(old_prob + 1e-10)
        p1 = ratio*advs
        p2 = K.clip(ratio, 1-self.clip_ratio, 1+self.clip_ratio)*advs
        # total loss = policy loss + entropy loss (entropy loss for promote action diversity)
        loss = -K.mean(K.minimum(p1,p2)+self.beta*(-y_pred*K.log(y_pred+1e-10)))
        return loss

    def predict(self, state):
        digits = self.actor.predict(state)
        #print("actor prediction", digits)
        return digits

    def fit(self,states,y_true,epochs,batch_size):
        self.actor.fit(states, y_true, epochs=epochs, verbose=0, shuffle=True, batch_size=batch_size,callbacks=[self.loss_printer])

    def save(self, path):
        self.actor.save_weights(path)

    def load(self, path):
        self.actor.load_weights(path)

"""
Critic net
"""
class Critic_Model:
    def __init__(self, input_shape, lr):
        self.critic = self.build_model(input_shape, lr)
        self.loss_printer = PrintLoss()

    def build_model(self, input_shape, lr):
        model = mlp_net(inputs_dim=input_shape, outputs_dim=1,outputs_activation="linear")
        model.compile(loss="mse",optimizer=keras.optimizers.Adam(learning_rate=lr))
        print(model.summary())
        return model

    def predict(self,state):
        digits = self.critic.predict(state)
        #print("critic prediction", digits)
        return digits

    def fit(self,states,y_true,epochs,batch_size):
        self.critic.fit(states, y_true, epochs=epochs, verbose=0, shuffle=True, batch_size=batch_size,callbacks=[self.loss_printer])

    def save(self, path):
        self.critic.save_weights(path)

    def load(self, path):
        self.critic.load_weights(path)

"""
A PPO agent class using images as input
"""
class PPOAgent:
    def __init__(
        self,
        state_dim,
        action_size,
        clip_ratio=0.2,
        lr_a=1e-3,
        lr_c=3e-3,
        beta=1e-3
    ):
        self.name = 'ppo_agent'
        self.action_size = action_size
        self.Actor = Actor_Model(state_dim,action_size,clip_ratio,lr_a,beta)
        self.Critic = Critic_Model(state_dim,lr_c)

    def action(self, state):
        pred = np.squeeze(self.Actor.predict(np.expand_dims(state,axis=0)), axis=0)
        act = np.random.choice(self.action_size,p=pred) # index of actions
        val = np.squeeze(self.Critic.predict(np.expand_dims(state,axis=0)), axis=0)
        # print("prediction, action, value:", pred, act, val)
        return pred, act, val

    def train(self, data, batch_size, iter_a=80, iter_c=80):
        states = data['states']
        actions = np.vstack(data['actions'])
        predictions = np.vstack(data['predictions'])
        advantages = np.vstack(data['advantages'])
        returns = np.vstack(data['returns'])
        # stack everything to numpy array
        y_true = np.hstack([advantages, predictions, actions])
        # training Actor and Crtic networks
        print("training Actor network...")
        self.Actor.fit(states, y_true, iter_a, batch_size)
        print("training Actor network...")
        self.Critic.fit(states, returns, iter_c, batch_size)

def getParameters():
    parser = argparse.ArgumentParser()
    parser.add_argument('--load', type=str, default=None)
    parser.add_argument('--vpsfile', type=str, default=None)
    parser.add_argument('--overlap', type=float, default=0.0) # control parameter for neighbors choice
    parser.add_argument('--max_ep', type=int, default=1000)
    parser.add_argument('--ep_step', type=int, default=50)
    parser.add_argument('--coverage', type=float, default=0.9)
    parser.add_argument('--ad', type=int, default=4)

    return parser.parse_args()

if __name__ == "__main__":
    args = getParameters()

    # load viewpoints
    vpGenerator = ViewPointGenerator()
    vps = vpGenerator.load(os.path.join(args.load, args.vpsfile))
    print("load {} viewpoints from file".format(len(vps)))
    # build neighborhood map
    util = ViewPointUtil2(vps=vps,overlap=args.overlap,ad=args.ad)
    util.buildNeighborMap()

    model_dir = os.path.join(sys.path[0],'..','saved',datetime.now().strftime("%Y-%m-%d-%H-%M"))
    print("model is saved to", model_dir)
    summary_writer = tf.summary.create_file_writer(model_dir)
    summary_writer.set_as_default()

    # create a environment
    buffer_cap = 600
    train_freq = 500
    env = CPPEnv(util, args.coverage)
    action_size = args.ad
    state_dim = (2,len(util.voxels)) # x,y,z of the vp and its neighborhood vps
    agent = PPOAgent(state_dim=state_dim, action_size=action_size, clip_ratio=0.2, lr_a=1e-4, lr_c=3e-4, beta=5e-3)
    buffer = ReplayBuffer(input_shape=state_dim, action_size=action_size, size=buffer_cap)

    t0 = datetime.now()

    bestvps = []
    bestScore = 0.0
    bestCoverage = 0.0
    success_counter = 0
    epIndices = []
    epReturns = []
    epCoverages = []
    epDists = []
    for ep in range(args.max_ep):
        vpIdx = np.random.randint(len(vps))
        vp,nbvps,obs,coverage = env.reset(vpIdx)
        traj = [vp]
        epReturn, epLength = 0, 0
        for step in range(args.ep_step):
            pred, act, val = agent.action(obs)
            vpIdx = nbvps[act]
            vp,nbvps,r,n_obs,coverage = env.step(vpIdx)
            traj.append(vp)

            buffer.store(obs,tf.one_hot(act,action_size).numpy(),r,pred,val)
            obs = n_obs
            epReturn += r
            epLength += 1
            if coverage > args.coverage:
                success_counter += 1
                break;

        epIndices.append(ep+1)
        epReturns.append(epReturn)
        epCoverages.append(coverage*100)
        epDists.append(env.distance())

        if epReturn > bestScore:
            bestvps = traj
            bestScore = epReturn
            bestCoverage = coverage

        tf.summary.scalar("episode total reward", epReturn, step=ep+1)
        print("Episode:{},Return:{:.4f},Length:{},Coverage:{:.4f},Best:{:.4f},Success:{}".format(ep+1, epReturn, epLength,coverage*100,bestCoverage*100,success_counter))

        buffer.ep_update(gamma=0.99, lamda=0.97)
        size = buffer.size()
        if size >= train_freq or (ep+1) == args.max_ep:
            print("ppo training with ",size," experiences...")
            agent.train(data=buffer.get(), batch_size=size, iter_a=80, iter_c=80)

    t1 = datetime.now()
    print("PPO find {} viewpoints for {:.2f}% coverage in {}.".format(len(bestvps), bestCoverage*100, str(t1-t0)))
    traj_file = os.path.join(sys.path[0],'..','trajectory/ppo_best.txt')
    vpGenerator.save(traj_file,alterTour(bestvps))

    # plot
    fig = go.Figure()
    title = "PPO - Total Reward and Coverage"
    fig.add_trace(go.Scatter(
        x = epIndices,
        y = smoothExponential(epReturns,0.995),
        # mode='lines+markers',
        name="Total Reward",
        marker=dict(color="#0075DC")
        # line = dict(shape = 'linear', color = "#0075DC", width = 1, dash = 'solid')
        ))
    fig.add_trace(go.Scatter(
        x = epIndices,
        y = smoothExponential(epCoverages,0.995),
        # mode='lines+markers',
        name="Coverage (%)",
        marker=dict(color="#552233")
        # line = dict(shape = 'linear', color = "#552233", width = 1, dash = 'dash')
        ))
    # fig.add_trace(go.Scatter(
    #     x = epIndices,
    #     y = smoothExponential(epDists,0.99),
    #     mode='lines+markers',
    #     name="Flying Distance (m)",
    #     marker=dict(color="#ff5511")
    #     # line = dict(shape = 'linear', color = "#ff22dd", width = 1, dash = 'dot')
    #     ))
    fig.update_layout(
        title=title,
        xaxis_title="Episode",
        yaxis_title="",
        font=dict(
            family="Arial",
            size=20,
            color="Black"
        ),
        plot_bgcolor="rgb(255,255,255)"
    )
    fig.show()
