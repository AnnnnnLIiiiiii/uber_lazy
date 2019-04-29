import cv2
import numpy as np
import math
import heapq as hpq
from P5_map import map_generate
from A_star_8_connected_space import map_initialize

grid_size = 5
map = map_generate()

start_point, goal_point, modified_map, map_show = map_initialize(map, grid_size)