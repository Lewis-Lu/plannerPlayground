#!

import cv2
import math
from matplotlib.pyplot import waitforbuttonpress
import numpy
from numpy.lib import select

class maper:
    def __init__(self, filename):
        try:
            self.im = cv2.imread(filename, 0)
        except:
            raise FileExistsError("Cannot import the file")
        
        self.filename = filename
        self.resolution = 0.05 #m
        self.shape = self.im.shape
        self.width = self.shape[0]
        self.height = self.shape[1]

    def getGridVal(self, x, y):
        return self.im[x, y]

    def degenerate(self, tar_res):
        cur_res = self.resolution
        srh_idc = math.floor(tar_res / cur_res)
        new_width = math.floor(self.width / srh_idc)
        new_height = math.floor(self.height / srh_idc)
        new_im = []
        for i in range(new_width):
            new_row = []
            for j in range(new_height):
                mat = self.im[srh_idc*i:srh_idc*(i+1)-1, srh_idc*j:srh_idc*(j+1)-1]
                if numpy.any(mat == 0):
                    new_row.append(0)
                else:
                    new_row.append(255)
            new_im.append(new_row)
        self.im = numpy.uint8(new_im)
        self.resolution = tar_res
        self.shape = self.im.shape
        self.width = new_width
        self.height = new_height
        print(f'Map is degenerating to {self.width}x{self.height} with resolution {self.resolution} m/pixel.')

    def preview(self):
        print(f'file path is [{self.filename}]')
        print(f'The resolution is {self.resolution} -- Height is {self.height} -- Width is {self.width}')
        cv2.namedWindow("Warehouse")
        # cv2.resizeWindow("Warehouse", 500, 500)
        cv2.imshow("Warehouse", self.im)
        cv2.imwrite("result.jpg", self.im)
        cv2.waitKey(0)
            

def mapUnitTest():
    warehouse = maper('maps/warehouse.pgm')
    warehouse.degenerate(1)
    warehouse.preview()

if __name__ == '__main__':
    mapUnitTest()
    