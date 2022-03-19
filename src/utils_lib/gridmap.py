#!/usr/bin/python3

import numpy as np
from bresenham import *

# Defining the functions to convert from Odds to log odds and viceversa


def l2p(l):
    return 1-(1/(1+np.exp(l)))


def p2l(p):
    return np.log(p/(1-p))


model_p_occ = 0.75
model_p_free = 0.45
model_p_prior = 0.5

model_l_occ = p2l(model_p_occ)
model_l_free = p2l(model_p_free)
model_l_prior = p2l(model_p_prior)


class GridMap:

    # Threshold values for the Log odss
    LMAX = 6.91
    LMIN = -6.91
    # center = center position (x,y) of the grid with respect to the fixed_frame
    # map_width = size of grid map in meters (width x width)
    # cell_size = size of each cell in meters (cell_size x cell_size )

    def __init__(self, center, cell_size=0.1, map_width=30):
        self.map_width = np.array([map_width, map_width])  # meters
        self.cell_size = cell_size  # meters
        self.grid = np.zeros(
            (int(map_width/cell_size), int(map_width/cell_size)))
        self.origin = np.array(center) - self.map_width/2
        map_rows = int(map_width/cell_size)
        map_cols = map_rows

    def get_map(self):
        return self.grid

    def update_cell(self, position, p):

        for i in range(len(position)-2):
            self.grid[position[i][1]][position[i][0]] = np.clip(
                self.grid[position[i][1]][position[i][0]] + p2l(1-p), self.LMIN, self.LMAX)

        self.grid[position[-1][1]][position[-1][0]] = np.clip(
            self.grid[position[-1][1]][position[-1][0]] + p2l(p), self.LMIN, self.LMAX)

    def add_ray(self, start, angle, range, p):

        # Extracting start position of the ray
        x0 = start[0]
        y0 = start[1]

        # Computing the end position of the ray
        x1 = start[0] + range*np.cos(angle)
        y1 = start[1] + range*np.sin(angle)

        # Converting the start [i0,j0] and end [i1,j1] positions in cells
        i0, j0 = self.position_to_cell([x0, y0])
        i1, j1 = self.position_to_cell([x1, y1])
        points = list(bresenham(int(i0), int(j0), int(i1), int(j1)))
        self.update_cell(points, p)

        return self.grid

    def position_to_cell(self, position):
        # Transform a position with respecto to the fixed_frame to a
        # cell index (rows and cols in the grid map)

        i = ((position[0]-self.origin[0])/self.cell_size)
        j = ((position[1]-self.origin[1])/self.cell_size)

        return i, j
