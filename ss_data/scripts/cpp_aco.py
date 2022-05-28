"""
discrete coverage path planning problem
it could be divided into two problems: first, solving the set covering problem to find a minimum ser of
viewpoints among a bounch of candidate viewpoints that fully cover the area of interest; second, solving
the traverling salesman problem to find a shortest visit path among the set of selected viewpoints.
"""
import random
import numpy as np
import math
import os
from util import *
import argparse
import copy
from map import *
import pandas as pd
from datetime import datetime

import plotly.graph_objects as go
from plotly.subplots import make_subplots

np.random.seed(124)

"""
Greedy Set Covering
The main idea of the algorithm is to cover the universe taking every time the apprently most
convenient set in the sets lists. In other words, every while cycle the program will search among
all sets and will take the one with the highest ratio bwteen the elements not yet covered and the
ralative cost of the set. This algorithm doesn't always give the best result, but certainly it
gives an optimal one.
"""
class SCPSolver:
    def __init__(self,vps,startIdx,coverage=1.0):
        self.vps = vps
        self.startIdx = startIdx
        self.coverage = coverage

    def computeMinimumCoveringViewpoints(self,iter=1):
        """
        Compute an approximate minimum set of viewpoints to cover the who valid grid
        """
        universe = set()
        subsets = []
        for i in range(len(self.vps)):
            voxelset = self.vps[i].voxels
            universe |= voxelset
            subsets.append(voxelset)

        start = subsets[self.startIdx]
        minCover = subsets
        for i in range(iter):
            cover = self.setCover(universe,copy.deepcopy(subsets),start)
            print("set covering greedy algorithm: iteration {} find {} viewpoints".format(i,len(cover)))
            if len(cover) < len(minCover):
                minCover = cover

        # duplicate grid
        coverage, duplicate1, duplicate2 = self.computeDuplication(universe,minCover)
        # return the view points
        minvps = []
        for s in minCover:
            index = subsets.index(s)
            minvps.append(self.vps[index])
        print("Find {} viewpoints in {} candidates with coverage {:.2f}%, duplication {:.2f}% {:.2f}%".format(len(minvps), len(self.vps), coverage, duplicate1, duplicate2))
        return minvps

    def setCover(self, universe, subsets, start):
        cover = []
        cover.append(start)
        covered = set()
        covered |= start
        # Greedily add the subsets (shuffled) with the most uncovered points
        random.shuffle(subsets)
        coverage = float(len(covered)) / float(len(universe))
        while coverage < self.coverage:
            subset = max(subsets, key=lambda s: len(s-covered))
            cover.append(subset)
            covered |= subset # union two sets
            coverage = float(len(covered)) / float(len(universe))
        return cover

    def computeDuplication(self,universe,cover):
        """
        duplicate1: minimum duplicaitoin with count overlap grid only once
        duplicate2: maximum duplicatioin with count grid whenever it overlap with another
        """
        intersectSet = set()
        duplicate1 = 0
        duplicate2 = 0
        for s1 in cover:
            for s2 in cover:
                if s1 != s2:
                    s = s1 & s2 #interestion
                    for e in s:
                        duplicate1 += 1
                        if e not in intersectSet:
                            intersectSet.add(e)
                            duplicate2 += 1

        allCover = set()
        for s in cover:
            allCover |= s

        total = len(universe)
        uncovered = len(universe)-len(allCover)
        coverage = float(total-uncovered)/float(total)
        duplicateRatio1 = float(duplicate1)/float(total)
        duplicateRatio2 = float(duplicate2)/float(total)
        return coverage*100, duplicateRatio1*100, duplicateRatio2*100

EPSILON = 1e-6
STAGNATION = 50

"""
In the ant colony optimization algorithms, an artificial ant is a simple computational agent that
searches for good solutions to a given optimization problem.
In the first step of each iteration, each ant stochastically constructs a solution, i.e. the order
in which the edges in the graph should be followed.
In the second step, the paths found by the different ants are compared. The last step consists of
updating the pheromone levels on each edge.

procedure ACO_MetaHeuristic is
    while not terminated do
        generateSolutions()
        daemonActions()
        pheromoneUpdate()
    repeat
end procedure

"""
class Ant:
    """
    Single Ant
    Create a single ant with its properties
    :param int size: the dimension or length of the ant
    """
    def __init__(self, size):
        self.size = size
        self.tour = np.ones(self.size, dtype=np.int64)*-1
        self.visited = np.zeros(self.size, dtype=np.int64)
        self.tourLength = np.inf

    def clone(self):
        """
        Returns a deep copy of the current Ant instance
        """
        ant = Ant(len(self.tour))
        ant.tour = self.tour.copy()
        ant.visited = self.visited.copy()
        ant.tourLength = self.tourLength
        return ant


class ACO:
    """
    The Ant Colony Optimization metaheuristic (MAX-MIN Ant System MMAS)
    :param vps: viewpoints
    :param startIdx: start viewpoint idx
    :param ants: number of ants in the colony
    :param alpha: the pheromone trail influence
    :param beta: the heuristic information influence
    :param rho: the pheromone evaporation parameter
    """
    def __init__(self, vps, startIdx=-1, ants = -1, alpha = 1.0, beta = 2.0, rho = 0.05):
        self.vps = vps
        self.startIdx = startIdx

        self.ants = ants
        self.alpha = alpha
        self.beta = beta
        self.rho = rho
        self.start = None
        self.initialize()

    def initialize(self):
        # initialize the problem
        self.n = len(self.vps)
        self.distMatrix = self.computeDistances(self.vps)
        self.CNN = self.computeNNTourLength(self.n, self.distMatrix) # initial tour
        # self.trail0 = self.ants / self.CNN # for origin AS
        self.trail0 = 1.0 / (self.rho * self.CNN)
        self.uGB = 1
        self.restartIter = 1
        self.trailMax = 1.0 / (self.rho * self.CNN)
        self.trailMin = self.trailMax / (2.0*self.n)

        self.createColony(self.ants, self.n)
        self.resetPheromone(self.trail0)
        self.eta = 1.0 / (self.distMatrix + 0.1) # heuristic infomation
        self.choiceInfo = np.empty_like(self.pheromone)
        self.choiceInfo[:] = self.pheromone
        self.computeChoiceInfo()



        self.iter = 0
        self.bestSoFarAnt = Ant(self.n)
        self.restartBestAnt = Ant(self.n)
        self.foundBest = 0 # iteration in which restart best ant is found
        self.restartFoundBest = 0

    def computeDistances(self, vps):
        """
        distance matrix of viewpoints
        the largest value assign to the city to itself
        """
        dim = len(vps)
        distMatrix = np.zeros((dim, dim))
        for i in range(dim):
            for j in range(dim):
                if i == j:
                    distMatrix[i][j] = MAXVALUE # give a large value for the same vp
                else:
                    distMatrix[i][j] = vpDistance(vps[i],vps[j])
        return distMatrix

    def computeNNTourLength(self, dim, distMatrix):
        """
        A TSP tour generated by the nearest-neighbor heuristic
        """
        tour = np.ones(dim, dtype=np.int64)*-1
        visited = np.zeros(dim, dtype=np.int64)
        r = self.startIdx # np.random.randint(0,dim) # initial to random city
        step = 0
        tour[step] = r
        visited[r] = 1
        while (step < dim-1):
            step+=1
            current = tour[step-1]
            # find next city, nearest neighbor (shortest distance)
            next = dim-1
            minDist = np.inf
            for v in range(dim):
                if not visited[v]:
                    dist = distMatrix[current][v]
                    if dist < minDist:
                        next = v
                        minDist = dist
            tour[step] = next
            visited[next] = 1
        return self.computeTourLength(tour)

    def computeTourLength(self,tour):
        return sum(vpDistance(self.vps[tour[i]],self.vps[tour[i+1]]) for i in range(len(tour)-1))

    def createColony(self, numOfAnts, size):
        """Create a colony of ants according to the number of ants specified,"""
        colony = []
        for i in range(numOfAnts):
            colony.append(Ant(size))
        self.colony = np.array(colony)
        return

    def resetPheromone(self, level=0.1):
        """Reset the pheromone to a default level"""
        self.pheromone = np.ones((self.n, self.n), dtype=np.float) * level
        return

    def computeChoiceInfo(self):
        """
        Compute the choice information matrix using the pheromone and heuristic information.
        """
        self.choiceInfo = self.pheromone ** self.alpha * self.eta ** self.beta
        return

    def evaporate(self):
        """
        Decrese the values of the pheromone trails on all the arcs by a constant factor rho.
        Matrix operations
        """
        self.pheromone = self.pheromone * (1.0-self.rho)
        return

    def updatePheromoneTrail(self):
        """
        Pheromone trails are evaporated and pheromones are deposited
        """
        self.evaporate()
        #self.updatePheromoneAS()

        self.updatePheromoneMMAS()
        self.checkPheromoneTrailLimits()

        self.computeChoiceInfo()
        return

    def updatePheromoneAS(self):
        """
        Manage global pheromone deposit for Ant System (origin)
        """
        for ant in self.colony:
            self.depositPheromone(ant)
        return

    def depositPheromone(self, ant):
        """
        Adds pheromone to the arcs belonging to the tours constructed by the ant
        """
        delta = 1.0 / ant.tourLength
        for i in range(self.n-1):
            j = ant.tour[i]
            k = ant.tour[i+1]
            self.pheromone[j][k] = self.pheromone[j][k] + delta
            self.pheromone[k][j] = self.pheromone[j][k]
        return

    def generateSolutions(self):
        """Construct valid solutions for the TSP."""
        step = 0
        # 1. Clear ants memory
        for ant in self.colony:
            for i in range(len(ant.visited)):
                ant.visited[i] = 0
        # 2. Assign an initial random city to each ant
        for ant in self.colony:
            r = self.startIdx #np.random.randint(0, self.n)
            ant.tour[step] = r
            ant.visited[r] = 1
        # 3. Each ant constructs a complete tour
        while step < self.n-1:
            step += 1
            for k in range(self.ants):
                self.decisionRule(k,step)
        # 4. Move to initial city and compute each ant's tour length
        for ant in self.colony:
            ant.tourLength = self.computeTourLength(ant.tour)

    def decisionRule(self, k, i):
        """
        The ants apply the Ant System (AS) action choice rule eq.3.2
        :param int k: ant identifier
        :param int i: counter for construction step
        """
        c = self.colony[k].tour[i-1] # current city
        # create a roulette wheel, like in evolutionary computation
        # sum the probabilities of the cities not yet visited
        sumProp = 0.0
        selectProb = np.zeros(self.n, dtype=np.float)
        for j in range(self.n):
            if self.colony[k].visited[j]:
                selectProb[j] = 0.0 # if city has been visited, its probability is zero
            else:
                # assign a slice to the roulette wheel, proportional to the weight of the associated choice
                selectProb[j] = self.choiceInfo[c][j]
                sumProp += selectProb[j]

        # Spin the roulette wheel
        # Random number from the interval [0, sumProb], corresponding to a uniform distribution
        r = sumProp*np.random.random_sample()
        j = 0
        p = selectProb[j]
        while p < r:
            j += 1
            p += selectProb[j]

        self.colony[k].tour[i] = j
        self.colony[k].visited[j] = 1
        return

    def updateStatistics(self):
        """
        Manage some statistical information about the trial, especially
        if a new best solution (best-so-far or restart-best) if found and
        adjust some parametyers if a new best solution is found
        """
        iterBestAnt = self.findBest()
        # Update best so far ant
        diff = self.bestSoFarAnt.tourLength - iterBestAnt.tourLength
        if diff > EPSILON:
            self.bestSoFarAnt = iterBestAnt.clone()
            self.restartBestAnt = iterBestAnt.clone()
            self.foundBest = self.iter
            self.restartFoundBest = self.iter

            # update min and max pheromone trails for MMAS
            self.trailMax = 1. / (self.rho * self.bestSoFarAnt.tourLength)
            self.trailMin = self.trailMax / (2. * self.n)
            self.trail0 = self.trailMax

        # Update restart best ant
        diff = self.restartBestAnt.tourLength - iterBestAnt.tourLength
        if diff > EPSILON:
            self.restartBestAnt = iterBestAnt.clone()
            self.restartFoundBest = self.iter
        return

    def findBest(self):
        """
        FInd the best ant object from the colony in the current iteration
        """
        best = self.colony[0]
        for ant in self.colony:
            if ant.tourLength < best.tourLength:
                best = ant.clone()
        return best

    def run(self, maxIter):
        progress = []
        print("*** Running Ant Colony Optimization ***")
        while self.iter < maxIter:
            self.generateSolutions()
            self.updateStatistics()
            self.updatePheromoneTrail()
            #self.searchControl() # for MMAS
            self.iter += 1
            # console output
            lenValues = np.array([ant.tourLength for ant in self.colony])
            progress.append(np.amin(lenValues))
            stats = [self.iter,np.amax(lenValues),np.amin(lenValues),np.mean(lenValues),np.std(lenValues)]
            print("{0}\t max {1:.4f} \t min {2:.4f} \t mean {3:.4f} \t std {4:.4f}".format(stats[0], stats[1], stats[2], stats[3], stats[4]))
        bestTour = self.bestSoFarAnt.tour.tolist()
        print("{} city tour with length {:.2f}".format(len(bestTour), self.computeTourLength(bestTour)))
        bestvps = [self.vps[i] for i in bestTour]
        return progress, bestvps

    """
    Below functions are for MMAS
    """
    def searchControl(self):
        """
        Occasionally compute some statistics and check whether or not if the algorithm is converged.
        Side effects: restart_best and best so far ant may be updated
        trail_min and trail max used by MMAS may be updated

        MAX-MIN Ant System was the first ACO algorithm to use pheromone trail re-initialisation
        for MMAS trail0 == trailMax
        """
        if not (self.iter % 100):
            self.restartBestAnt.tourLength = np.inf
            self.resetPheromone(self.trail0)
            self.computeChoiceInfo()
            self.restartIter = self.iter
        return

    def updatePheromoneMMAS(self):
        """
        Manage global pheromone deposit for MAX-MIN Ant System
        """
        if (self.iter % self.uGB): # update with iterBestAnt
            iterBestAnt = self.findBest()
            self.depositPheromone(iterBestAnt)
        else:
            noImprov = self.iter - self.restartFoundBest
            if (self.uGB == 1 and noImprov > STAGNATION):
                self.depositPheromone(self.bestSoFarAnt)
            else:
                self.depositPheromone(self.restartBestAnt)
        return

    def checkPheromoneTrailLimits(self):
        """
        Only for MMAS without local search: keep pheromone trails inside trail limits.
        pheromones are forced to interval [trailMin, trailMax]
        """
        self.pheromone = np.clip(self.pheromone, self.trailMin, self.trailMax)
        return


def getParameters():
    parser = argparse.ArgumentParser()
    parser.add_argument('--load', type=str, default=None)
    parser.add_argument('--vpsfile', type=str, default=None)
    parser.add_argument('--scIter', type=int, default=1)
    parser.add_argument('--tc', type=float, default=1.0)
    parser.add_argument('--ants', type=int, default=-1)
    parser.add_argument('--acIter', type=int, default=200)
    parser.add_argument('--alpha', type=float, default=1.0)
    parser.add_argument('--beta', type=float, default=2.0)
    parser.add_argument('--rho', type=float, default=0.5)
    return parser.parse_args()

# main loop
if __name__ == '__main__':
    args = getParameters()

    vpGenerator = ViewPointGenerator()
    vps = vpGenerator.load(os.path.join(args.load, args.vpsfile))

    startIdx = 0
    scp = SCPSolver(vps,startIdx,coverage=args.tc)
    minvps = scp.computeMinimumCoveringViewpoints(iter=args.scIter)
    # vpGenerator.save(os.path.join(args.load, args.trajfile),minvps)

    t0 = datetime.now()
    # use ACO to solve TSP
    ants = args.ants
    if ants < 0:
        ants = len(minvps)
    tspACO = ACO(vps=minvps, startIdx=startIdx, ants=ants, alpha=args.alpha, beta=args.beta, rho=args.rho)
    progress, bestvps = tspACO.run(args.acIter)
    t1 = datetime.now()

    # save trajectory
    print("ACO find {} viewpoints for {:.2f}% coverage in {}.".format(len(bestvps), args.tc*100.0, str(t1-t0)))
    traj_file = os.path.join(sys.path[0],'..','trajectory/aco_best.txt')
    vpGenerator.save(traj_file, alterTour(bestvps))

    # plot
    iter = [i+1 for i in range(len(progress))]
    fig = go.Figure()
    title = "ACO - Traveling Distance"
    fig.add_trace(go.Scatter(
        x = iter,
        y = smoothExponential(progress,0.995),
        # mode='lines+markers',
        name="Traveling Distance",
        marker=dict(color="#552233")
        # line = dict(shape = 'linear', color = "#552233", width = 1, dash = 'dash')
        ))
    fig.update_layout(
        title=title,
        xaxis_title="Iteration",
        yaxis_title="Traveling Distance (m)",
        font=dict(
            family="Arial",
            size=20,
            color="Black"
        ),
        plot_bgcolor="rgb(255,255,255)"
    )
    fig.show()
