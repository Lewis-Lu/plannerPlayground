#!
import cv2
import math
import numpy as np
import matplotlib.pyplot as plt

class maper:
    def __init__(self, filename):
        try:
            self.im = cv2.imread(filename, 0) # numpy.ndarray
        except:
            raise FileExistsError(f"Cannot import the file {filename}")
        self.filename = filename
        self.resolution = 0.05 # original resolution
        self.shape = self.im.shape
        # opencv image coordinate [#row, #column] (row major in this case)
        # ---- columns (width) ---->
        # |
        # |
        # rows (height)
        # |
        # |
        # v
        self.rows = self.shape[0]
        self.cols = self.shape[1]
    
    def rowsInMeter(self):
        '''return the rows in meter'''
        return self.rows * self.resolution

    def ColumnsInMeter(self):
        '''return the cols in meter'''
        return self.cols * self.resolution

    def degenerate(self, tar_res):
        '''degenerate the resolution of the map'''
        srh_idc = math.floor(tar_res / self.resolution)
        new_rows = math.floor(self.rows / srh_idc)
        new_cols = math.floor(self.cols / srh_idc)
        new_im = np.ndarray((new_rows, new_cols), dtype='uint8') # row-major
        for i in range(new_rows):
            for j in range(new_cols):
                mat = self.im[srh_idc*i:srh_idc*(i+1)-1, srh_idc*j:srh_idc*(j+1)-1]
                if np.any(mat == 0):
                    new_im[i,j] = 0
                else:
                    new_im[i,j] = 255
        self.im = new_im
        self.resolution = tar_res
        self.shape = self.im.shape
        self.rows = new_rows
        self.cols = new_cols
        print(f'Map is degenerating to [{self.rows}, {self.cols}] with resolution {self.resolution} m/pixel.')

    def __coordMap2plot(self, m):
        return [m[1], self.rows-1-m[0]]

    def createCartesianGridMap(self):
        '''create a cartesian grid map for planning and plotting'''
        print("Creating the cartesian grid map...")
        new_im = np.zeros((self.cols, self.rows), dtype='uint8') # all black
        for r in range(self.rows):
            for c in range(self.cols):
                cart_x, cart_y = self.__coordMap2plot([r, c])
                if self.im[r][c] != 0:
                    new_im[cart_x][cart_y] = 255
        self.cartIm = new_im
        self.xlim = [0, self.cols]
        self.ylim = [0, self.rows]
        print(f"Cartesian gird map built: xlim:{self.xlim}, ylim:{self.ylim}")

    def cartesianResult(self, startInMeter, goalInMeter, path):
        fig = plt.figure()
        ax = plt.subplot(111)
        plt.axis("equal")
        start = [int(startInMeter[0]/self.resolution)-1, int(startInMeter[1]/self.resolution)-1]
        goal = [int(goalInMeter[0]/self.resolution)-1, int(goalInMeter[1]/self.resolution)-1]
        # plot the env
        for x in range(self.xlim[0], self.xlim[1]):
            for y in range(self.ylim[0], self.ylim[1]):
                if self.cartIm[x][y] == 0:
                    ax.fill_between([x, x+1], y, y+1, facecolor='black', linestyle='-')
        # set the start point 
        ax.fill_between([start[0], start[0]+1], start[1], start[1]+1, facecolor='green', linestyle='-')
        # set the goal point
        ax.fill_between([goal[0], goal[0]+1], goal[1], goal[1]+1, facecolor='red', linestyle='-')
        for p in path:
            ax.fill_between([p[0], p[0]+1], p[1], p[1]+1, facecolor='blue', linestyle='-')
        plt.ylim(self.ylim[0], self.ylim[1])
        plt.xlim(self.xlim[0], self.xlim[1])
        plt.show()

    def preview(self):
        print(f'file path is [{self.filename}]')
        print(f'The resolution is {self.resolution} -- #Row is {self.rows} -- #Col is {self.cols}')        
        cv2.namedWindow("Warehouse")
        # cv2.resizeWindow("Warehouse", 500, 500)
        cv2.imshow("Warehouse", self.im)
        cv2.waitKey(0)


def mapUnitTest():
    warehouse = maper('maps/warehouse.pgm')
    warehouse.degenerate(1)
    warehouse.createCartesianGridMap()
    warehouse.cartesianResult([5,10], [10,20], [])
    warehouse.preview()

if __name__ == '__main__':
    mapUnitTest()
    