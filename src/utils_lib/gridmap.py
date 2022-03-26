#!/usr/bin/python3

import numpy as np
from .bresenham import *


def log_ods(p): return np.log(p/(1-p))


class GridMap:

    LMAX = 6.91
    LMIN = -6.91

    def __init__(self, center, cell_size=0.1, map_width=30):
        self.map_width = np.array([map_width, map_width])
        self.cell_size = cell_size
        self.grid = np.zeros(
            (int(map_width/cell_size), int(map_width/cell_size)))
        self.origin = np.array(center) - self.map_width/2

    def get_map(self):
        return self.grid

    def update_cell(self, position, p):

        for i in range(0, len(position)-1):
            self.grid[position[i][1]][position[i][0]] = np.clip(
                self.grid[position[i][1]][position[i][0]] + log_ods(1-p), self.LMIN, self.LMAX)

        self.grid[position[-1][1]][position[-1][0]] = np.clip(
            self.grid[position[-1][1]][position[-1][0]] + log_ods(p), self.LMIN, self.LMAX)

    def add_ray(self, start, angle, range, p):

        # Starting Postions
        x_not = start[0]
        y_not = start[1]

        # Final Positions
        x1 = start[0] + range*np.cos(angle)
        y1 = start[1] + range*np.sin(angle)

        start_x, start_y = self.position_to_cell([x_not, y_not])
        end_x, end_y = self.position_to_cell([x1, y1])
        bres_start = [int(start_x), int(start_y)]
        bres_end = [int(end_x), int(end_y)]
        points = list(bresenham_line(bres_start, bres_end, 1))
        self.update_cell(points, p)

        return self.grid

    def position_to_cell(self, position):
        row_position = ((position[0]-self.origin[0])/self.cell_size)
        col_position = ((position[1]-self.origin[1])/self.cell_size)

        return row_position, col_position
