
from enum import Enum
import numpy as np

"""
This file contains all the constants and parameters of the simulator.
It comes handy when you want to make one shot simulations, making parameters and constants vary in every
simulation. For an extensive experimental campaign read the header at src.simulator.
"""

# ----------------------------- SIMULATION PARAMS ---------------------------- #

SIM_SEED = 0                # int: seed of this simulation.
SIM_DURATION = 3000000      # int: steps of simulation. (np.inf)
SIM_TS_DURATION = 0.150     # float: seconds duration of a step in seconds.

ENV_WIDTH = 1500      # float: meters, width of environment
ENV_HEIGHT = 1500     # float: meters, height of environment

N_DRONES = 2          # int: number of drones.
N_OBSTACLES = 10      # number of random obstacles in the map
N_GRID_CELLS = 3      # number of cells in the grid

# IMPORTANT: coordinates of the drones at the beginning, it can be NONE in that case drone will follow
# fixed tours determined in FIXED_TOURS_DIR
INITIAL_DRONE_COORDS = [ENV_WIDTH / 2, ENV_HEIGHT]

DRONE_SPEED = 5               # float: m/s, drone speed.
DRONE_ANGLE = 0               # degrees (0, 359)
DRONE_SPEED_INCREMENT = 5     # increment at every key stroke
DRONE_ANGLE_INCREMENT = 45    # increment at every key stroke
DRONE_COM_RANGE = 100         # float: meters, communication range of the drones.
DRONE_SENSING_RANGE = 0       # float: meters, the sensing range of the drones.
DRONE_MAX_BUFFER_SIZE = 0     # int: max number of packets in the buffer of a drone.
DRONE_MAX_ENERGY = 100        # int: max energy of a drone.
DRONE_RADAR_RADIUS = 60       # meters

# base station
BASE_STATION_COORDS = [ENV_WIDTH / 2, 0]   # coordinates of the base staion
BASE_STATION_COM_RANGE = 200               # float: meters, communication range of the depot.

# map
PLOT_TRAJECTORY_NEXT_TARGET = True

# ------------------------------- CONSTANTS ------------------------------- #

FIXED_TOURS_DIR = "data/tours/"        # str: the path to the drones tours
HANDCRAFTED_PATH = False

PLOT_SIM = True       # bool: whether to plot or not the simulation (set to false for faster experiments)
WAIT_SIM_STEP = 0     # float >= 0: seconds, pauses the rendering for x seconds
SKIP_SIM_STEP = 5     # int > 0 : steps, plot the simulation every x steps
DRAW_SIZE = 700       # int: size of the drawing window

SAVE_PLOT = False              # bool: whether to save the plots of the simulation or not
SAVE_PLOT_DIR = "data/plots/"  # string: where to save plots

TARGETS_COORDS = [(750, 750)]
