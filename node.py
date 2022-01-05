#!

# This is the class that specifies the Node class in the Graph Search Task

# Author: Lu, Hong
# Email: luh.lewis@gmail.com
# Created: 2021, Nov, 24
# Last Modified: 2021, Dec, 31

import math

class node:
    def __init__(self, coord) -> None:
        """ initialization of the class Node """
        try:
            self._x = coord[0]
            self._y = coord[1]
        except:
            raise AssertionError("The dimension of the the node should be 2")
        self._G = 0
        self._F = 0
        self._isVisited = False
        self._isOpen = False
        self._isClosed = False
        self._predecessor = None

    def __repr__(self) -> str:
        pass

    def __str__(self):
        return f'Node @ Grid({self._x},{self._y})--G({self._G})--F({self._F})'

    def __lt__(self, that):
        ''' less than comparison used in the heap '''
        return self.F < that.F

    def __eq__(self, __o: object) -> bool:
        return self._x == __o._x and self._y == __o._y

    @property
    def X(self):
        return self._x

    @property
    def Y(self):
        return self._y

    @property
    def G(self):
        return self._G

    @G.setter
    def G(self, gvalue):
        self._G = gvalue

    @property
    def F(self):
        return self._F

    @G.setter
    def F(self, fvalue):
        self._F = fvalue

    @property
    def isVisited(self):
        return self._isVisited

    @isVisited.setter
    def isVisited(self, val):
        self._isVisited = val

    def heurisitic_manhattan(self, that: object):
        return abs(self._x - that._x) + abs(self._y - that._y)

    def heuristic_euclidean(self, that: object):
        return math.sqrt((self._x - that._x)**2 + (self._y - that._y)**2)


def nodeUnitTest():
    a = node([1, 1])
    b = node([3, 1])
    print(a == b)
    print(a.heurisitic_manhattan(b))

if __name__ == '__main__':
    nodeUnitTest()
