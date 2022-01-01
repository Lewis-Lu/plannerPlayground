#!
# identify the intepreter for python

# Author: Lu, Hong
# Email: luhong@mooe-robot.com
# Created: 2021, Nov, 24
# Last Modified: 2021, Nov, 24

from heapq import heappush, heappop, heapify
import numpy as np
import matplotlib.pyplot as plt
from node import node
from mapping import maper

class Astar:
    def __init__(self, mapFilename, start, goal) -> None:
        self.map = maper(mapFilename)
        self.map.degenerate(1) # set resolution to 0.1m
        self._resolution = self.map.resolution
        self.min_x = 0
        self.min_y = 0
        self.max_x = self.map.width
        self.max_y = self.map.height
        self._start = [np.uint8(start[0]/self._resolution), np.uint8(start[1]/self._resolution)]
        self._goal = [np.uint8(goal[0]/self._resolution), np.uint8(goal[1]/self._resolution)]
        self._nnDir = [[1,0],[0,-1],[-1,0],[0,1]]
        self._nodeDict = {}
        self._nodeVisitNum = 0
        self._path = []

    @property
    def start(self):
        return self._start
    
    @start.setter
    def start(self, val):
        if(len(val) != 2):
            raise ValueError("the dimension of the start should be 2")
        self._start = val
        
    @property
    def goal(self):
        return self._goal
    
    @goal.setter
    def goal(self, val):
        if(len(val) != 2):
            raise ValueError("the dimension of the goal should be 2")
        self._goal = val

    def getPath(self):
        return self._path.reverse()

    def isGoal(self, cur: object, goal: object) -> bool:
        return cur == goal

    def findNeighbours(self, cur: object) -> list:
        x = cur._x
        y = cur._y
        nn = []
        for trans in self._nnDir:
            nn_tmp = [trans[0] + x, trans[1] + y]
            # sanity check
            if self.map.getGridVal(nn_tmp[0], nn_tmp[1]) != 0 and nn_tmp[0] > self.min_x and nn_tmp[0] < self.max_x and nn_tmp[1] > self.min_y and nn_tmp[1] < self.max_y:
                nn.append(nn_tmp)
        return nn

    def plan(self) -> object:
        startNode = node(self._start)
        openSet = []
        heappush(openSet, startNode)
        while openSet:
            current = heappop(openSet)
            self._nodeDict[current._x * self.max_x + current._y] = current
            self._nodeVisitNum += 1
            # reach the goal
            if self.isGoal(current, node(self._goal)):
                return current
            current._isClosed = True
            current._isVisited = True
            # explore the neighbours
            nn = self.findNeighbours(current)
            if len(nn) == 0:
                # no neighbours, pop the next from the priority queue
                continue
            for n in nn:
                ngbNode = node(n)
                ngbNode.G = current.G + 1
                ngbNode.F = ngbNode.G + ngbNode.heurisitic_manhattan(node(self._goal))
                if not self._nodeDict.get(n[0] * self.max_x + n[1]):
                    # neighbour not visited, add to the dictionary and update its predecessor
                    heappush(openSet, ngbNode)
                    self._nodeDict[n[0] * self.max_x + n[1]] = ngbNode
                    ngbNode._predecessor = current
                    ngbNode._isVisited = True
                else:
                    # neighbout visited, update if can
                    nd = self._nodeDict[n[0] * self.max_x + n[1]]
                    if nd.F < ngbNode.F:
                        continue
                    else:
                        nd.F = ngbNode.F
                        nd._predecessor = current
        return None

    def retrievePath(self):
        res = self.plan()
        if res:
            while res:
                
                self._path.append([res._x, res._y])
                res = res._predecessor
            
    def plotGridEnv(self):
        fig = plt.figure()
        ax = plt.subplot(111)
        plt.axis("equal")
        # plt.grid(True, linestyle='-')

        plot_margin = 5;

        plt.ylim(self.min_y - plot_margin, self.max_y + 1 + plot_margin)
        plt.xlim(self.min_x - plot_margin, self.max_x + 1 + plot_margin)

        # for i in range(len(self.obstacle)):
        #     # define x linspace
        #     x = np.linspace(self.obstacle[i][0], self.obstacle[i][0] + self.resolution, 2)
        #     ax.fill_between(x, self.obstacle[i][1], self.obstacle[i][1] + self.resolution, facecolor='black', linestyle='-', edgecolor='black')
        
        for i in self._path:
            x = np.linspace(i[0], i[0] + self.resolution, 2)
            ax.fill_between(x, i[1], i[1] + self.resolution, facecolor='yellow', linestyle='-', edgecolor='black')

        # set the start point 
        x = np.linspace(self._start[0], self._start[0] + self._resolution, 2)
        ax.fill_between(x, self._start[1], self._start[1] + self._resolution, facecolor='green', linestyle='-', edgecolor='black')

        # set the goal point
        x = np.linspace(self._goal[0], self._goal[0] + self._resolution, 2)
        ax.fill_between(x, self._goal[1], self._goal[1] + self._resolution, facecolor='blue', linestyle='-', edgecolor='black')
        plt.show()
    

    def convertToCV(self, path) -> None:
        pathGreyColor = 125
        for p in path:
            self.map.im[p.X, p.Y] = pathGreyColor
        self.map.preview()


def main():
    print('Astar planner started.')
    planner = Astar('maps/warehouse.pgm', [10,10], [30,30])
    endNode = planner.plan()
    if endNode:
        print("Goal Reached")
        print(endNode)
        cur = endNode
        path = []
        while cur:
            print(cur)
            path.append(cur)
            cur = cur._predecessor
    planner.convertToCV(path)

if __name__ == '__main__':
    main()
