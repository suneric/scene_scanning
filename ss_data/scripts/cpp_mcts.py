"""
discrete coverage path planning problem
with monte carlo tree search algorithm
"""
import time
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

np.random.seed(124)

"""
State represent the visited viewpoints and covered grids
"""
class MCTSState(object):
    def __init__(self,util,currVp,vpsState,voxelState,cover,traveledDist=0.0):
        self.util = util
        self.currVp = currVp
        self.vpsState = vpsState
        self.voxelState = voxelState
        self.cover = cover
        self.traveledDist = traveledDist # total distance
        self.nbvps = self.util.neighbors(currVp.id)
        self.coverage = float(cover)/float(len(self.util.voxels))

    def isGameOver(self):
        return self.coverage == 1.0 or len(self.neighbors()) == 0

    def score(self):
        """
        return a score indicating how good the state is
        considering coverage, overlap and traveled distance
        """
        s = 100*self.coverage-0.1*self.traveledDist
        return s

    def neighbors(self):
        """
        return neighbor viewpoints which have not been visited
        """
        unvisited = []
        for i in range(len(self.nbvps)):
            vpIdx = self.nbvps[i]
            if not self.vpsState[vpIdx]:
                unvisited.append(vpIdx)
        return unvisited

    def move(self, nextVp):
        """
        move to next vp and return a new state
        """
        # copy viewpoints and grids tates
        vpsState = copy.deepcopy(self.vpsState)
        voxelState = copy.deepcopy(self.voxelState)

        # update viewpoints and grids states
        vpsState[nextVp.id] = 1

        newCover = 0
        for v in nextVp.voxels:
            if voxelState[v] == 0:
                voxelState[v] = 1
                newCover += 1

        # move to a new state
        cover = self.cover + newCover

        if self.currVp.id == nextVp.id:
            dist = 100 # give a large distance for the same viewpoint
        else:
            dist = vpDistance(self.currVp, nextVp)

        dist = self.traveledDist + vpDistance(self.currVp, nextVp)
        return MCTSState(self.util,nextVp,vpsState,voxelState,cover,dist)


# base Node
class MCTSNode(object):
    def __init__(self, util, state, parent=None):
        self.util = util
        self.state = state
        self.untriedVps = self.state.neighbors()

        self.totalReward = 0.
        self.numberOfVisit = 0.
        self.maxReward = 0.

        self.parent = parent
        self.children = []

    def viewpoint(self):
        """
        corresponding viewpoint
        """
        return self.state.currVp

    def isTerminalNode(self):
        return self.state.isGameOver()

    def isFullyExpanded(self):
        return len(self.untriedVps) == 0

    def best_child(self, c, e):
        """
        return a child node with control parameter and eplison
        """
        if np.random.rand() > e:
            weights = [child.q(c) for child in self.children]
            return self.children[np.argmax(weights)]
        else:
            return self.children[np.random.randint(len(self.children))]

    def q(self,c):
        """
        q value of node based on max reward, average reward
        """
        return c*self.maxReward + (1-c)*(self.totalReward/self.numberOfVisit)

    def rolloutPolicy(self, possibleMoves):
        return possibleMoves[np.random.randint(len(possibleMoves))]

    def rollout(self):
        """
        rollout: run a simulation with randomly choose a child to visit
        return a value of state score

        """
        cState = self.state
        score = cState.score()
        while not cState.isGameOver():
            nbvps = cState.neighbors()
            vpIdx = self.rolloutPolicy(nbvps)
            vp = self.util.viewpoints[vpIdx]
            cState = cState.move(vp)
            score = cState.score()
        # print("===rollout score {:.2f} ===".format(score))
        return score

    def expand(self):
        """
        expand the child by randomly choose a untried vp to visit
        """
        vpIdx = self.untriedVps.pop(np.random.randint(len(self.untriedVps)))
        vp = self.util.viewpoints[vpIdx]
        nextState = self.state.move(vp)
        child = MCTSNode(self.util, nextState, parent=self)
        self.children.append(child)
        return child

    def backpropagate(self,reward):
        self.numberOfVisit += 1.
        self.totalReward += reward
        if self.maxReward < reward:
            self.maxReward = reward
        if self.parent:
            self.parent.backpropagate(reward)


class MonteCarloTreeSearch(object):
    def __init__(self, util, node, cparam, decay, targetCoverage):
        self.root = node
        self.epsilon = 1.0
        self.cparam = cparam
        self.decay = decay
        self.targetCoverage = targetCoverage
        self.util = util

    def decayEpsilon(self, finalEpsilon=0.1):
        """
        a epsilon with a decay rate until a final eplison is reached
        """
        self.epsilon *= self.decay
        if self.epsilon <= finalEpsilon:
            self.epsilon = finalEpsilon
        return self.epsilon

    def treePolicy(self):
        """
        Tree policy: build up a a tree
        return a best child if the node is fully expand or a random
        otherwise expand the node with a simulation (rollout, Mento Carlo Method)
        """
        node = self.root
        while not node.isTerminalNode():
            if node.isFullyExpanded():
                node = node.best_child(self.cparam, self.epsilon)
            else:
                return node.expand()
        return node

    def search(self,iteration,fe):
        progress = []
        for i in range(iteration):
            self.decayEpsilon(finalEpsilon=fe)
            v = self.treePolicy()
            r = v.rollout()
            v.backpropagate(r)
            vps, coverage = self.test()
            progress.append(coverage)
            print("iteration {}, epsilon {:.4f}, viewpoints explore {}, coverage {:.2f}%".format(i,self.epsilon,len(vps),coverage*100))
            if coverage >= self.targetCoverage:
                print("desired coverage achieved {:.2f}%".format(self.targetCoverage*100))
                break
        return self.root.best_child(self.cparam, 0.0), progress

    def test(self):
        """
        Test the tree
        """
        bestvps = []
        node = self.root
        bestvps.append(self.util.originalViewpoint(node.viewpoint()))
        coverage = node.state.coverage
        while not node.isTerminalNode():

            if node.isFullyExpanded():
                node = node.best_child(c=self.cparam,e=0.0)
                bestvps.append(self.util.originalViewpoint(node.viewpoint()))
                coverage = node.state.coverage
            else:
                break

        return bestvps, coverage


"""
reset / initial state
"""
def initialState(util, startVp):
    vpsState = [0]*len(util.viewpoints)
    vpsState[startVp.id] = 1
    voxelState = [0]*len(util.voxels)
    for v in startVp.voxels:
        voxelState[v] = 1
    state = MCTSState(util,startVp,vpsState,voxelState,0,0.0)
    return state

############################################################
# main
def getParameters():
    parser = argparse.ArgumentParser()
    parser.add_argument('--load', type=str, default=None)
    parser.add_argument('--vpsfile', type=str, default=None)
    parser.add_argument('--overlap', type=float, default=0.1) # control parameter for neighbors choice
    parser.add_argument('--sn', type=int, default=50) # simulation count
    parser.add_argument('--ad', type=int, default=4) # action dimenstion, how many neighbors
    parser.add_argument('--coverage', type=float, default=0.9) # target coverage
    parser.add_argument('--cp', type=float, default=0.38) # control param q value
    parser.add_argument('--dr', type=float, default=0.998) # decay rate
    parser.add_argument('--fe', type=float, default=0.1) # final epsilon

    return parser.parse_args()

if __name__ == "__main__":
    args = getParameters()

    vpGenerator = ViewPointGenerator()
    vps = vpGenerator.load(os.path.join(args.load, args.vpsfile))
    print("load {} viewpoints from file".format(len(vps)))
    util = ViewPointUtil2(vps=vps,overlap=args.overlap,ad=args.ad)
    util.buildNeighborMap()

    # monte carlo tree search
    startIdx = 0 #np.random.randint(len(vps))
    startVp = util.viewpoints[startIdx]
    initState = initialState(util, startVp)
    root = MCTSNode(util,initState,parent=None)
    mcts = MonteCarloTreeSearch(util,root,cparam=args.cp,decay=args.dr,targetCoverage=args.coverage)
    node, progress = mcts.search(iteration=args.sn,fe=args.fe)
    bestvps, coverage = mcts.test()

    # save trajectory
    print("Monte Carlo Tree Search find {} viewpoints for {:.2f}% coverage.".format(len(bestvps), coverage*100))
    traj_file = os.path.join(sys.path[0],'..','trajectory/mcts_best.txt')
    vpGenerator.save(traj_file,alterTour(bestvps))

    # save training statistics
    # statFile = os.path.join(args.load, "mcts.csv")
    # dict = {'Iteration': [i for i in range(len(progress))], 'Coverage': progress}
    # df = pd.DataFrame(dict)
    # df.to_csv(statFile)
