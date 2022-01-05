#!
# identify the intepreter for python

# Author: Lu, Hong
# Email: luhong@mooe-robot.com
# Created: 2021, Nov, 24
# Last Modified: 2021, Nov, 24

from heapq import heappush, heappop, heapify
import numpy as np
from node import node
from mapping import maper
import param
import os

class Astar:
    def __init__(self, mapFilename, resolution) -> None:
        self.map = maper(mapFilename)
        self.map.degenerate(resolution)
        self.map.createCartesianGridMap()
        self._resolution = self.map.resolution
        self._xlimCartesian = self.map.xlim
        self._ylimCartesian = self.map.ylim
        self._start = [int(param.start[0]/self._resolution)-1, int(param.start[1]/self._resolution)-1]  # convert coordinate into the pixel
        self._goal = [int(param.goal[0]/self._resolution)-1, int(param.goal[1]/self._resolution)-1] # convert coordinate into the pixel
        self._nnDir = [[1,0],[0,-1],[-1,0],[0,1]]
        self._nodeDict = {}
        self._nodeVisitNum = 0
        self._path = []

    def getPath(self):
        return self._path.reverse()

    def isGoal(self, cur: object, goal: object) -> bool:
        """cur and goal is the object of the class node"""
        return cur == goal

    def findNeighbours(self, cur: object) -> list:
        x = cur.X # obtain the x coordinate through getter function
        y = cur.Y # obstain the y coordinate through getter function
        nn = [] # list for neighbours
        for dir in self._nnDir:
            nn_tmp = [dir[0] + x, dir[1] + y]
            if nn_tmp[0] >= self._xlimCartesian[0] and nn_tmp[0] < self._xlimCartesian[1] and nn_tmp[1] >= self._ylimCartesian[0] and nn_tmp[1] < self._ylimCartesian[1]:
                if self.map.cartIm[nn_tmp[0], nn_tmp[1]] != 0:
                    nn.append(nn_tmp)
        return nn

    def planOnCartesian(self) -> object:
        # # build up the cartesian x-y limits
        min_x = self._xlimCartesian[0]
        max_x = self._xlimCartesian[1]
        min_y = self._ylimCartesian[0]
        max_y = self._ylimCartesian[1]

        startNode = node(self._start) # startNode Initialization
        goalNode = node(self._goal) # goalNode Initialization
        startNode.F = startNode.heurisitic_manhattan(goalNode) # F-score for the startnode equals to the heuristic value
        openSet = []
        heappush(openSet, startNode)
        while openSet:
            # print("**********************")
            # for p in openSet:
            #     print(p)
            # print("current node:")
            # print(openSet[0])
            # print("**********************")
            # os.system("pause")

            heapify(openSet)
            current = heappop(openSet) # the node in the openSet with lowest F-score
            currentIdx = current.Y*max_x + current.X
            self._nodeDict[currentIdx] = current
            self._nodeVisitNum += 1
            if self.isGoal(current, goalNode): # reach the goal
                return current
            # current._isClosed = True
            # explore the neighbours
            nn = self.findNeighbours(current)
            if len(nn) == 0:
                # no neighbours for current node, search in the queue
                continue
            # for each neighbour of the current node
            for n in nn:
                ngbNode = node(n)
                ngbNode.G = current.G + 1
                ngbNode.F = ngbNode.G + ngbNode.heurisitic_manhattan(goalNode)
                nIndex = n[1]*max_x + n[0]
                try:
                    # neighbour visited before
                    tentative_nd = self._nodeDict[nIndex]
                    if tentative_nd.F < ngbNode.F:
                        continue
                    else:
                        tentative_nd.G = ngbNode.G
                        tentative_nd.F = tentative_nd.G + tentative_nd.heurisitic_manhattan(goalNode)
                        tentative_nd._predecessor = current
                        self._nodeDict[nIndex] = tentative_nd # update
                except:
                    # neighbour not visited, add to the dictionary and update its predecessor
                    heappush(openSet, ngbNode)
                    self._nodeDict[nIndex] = ngbNode # add new neighbour into the dictionary
                    ngbNode._predecessor = current
                    # ngbNode._isVisited = True
        return None

    def retrievePath(self):
        res = self.planOnCartesian()
        if res:
            while res:
                self._path.append([res._x, res._y])
                res = res._predecessor
        else:
            raise RuntimeError("No path")

def extractVisited(dict):
    return dict.values()

def main():
    print('Astar planner started.')
    planner = Astar('maps/warehouse.pgm', 0.1)
    planner.retrievePath()
    path = planner._path
    planner.map.cartesianResult(param.start, param.goal, path, extractVisited(planner._nodeDict))
    print(f"#node visited = {planner._nodeVisitNum}")
    
if __name__ == '__main__':
    main()
