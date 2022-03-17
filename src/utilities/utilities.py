
""" To clean. """

from src.utilities import config

import pathlib
import time
import json
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import pickle
from ast import literal_eval as make_tuple

from shapely.geometry import LineString




def log(message_to_log, is_to_log=True, current_ts=1, log_every=1):
    """ Logs message_to_log, if is_to_log or every log_every steps (given current_ts). """
    if not is_to_log or not (current_ts % log_every == 0):
        return
    print(message_to_log)


def current_date():
    return str(time.strftime("%d%m%Y-%H%M%S"))


def euclidean_distance(p1, p2):
    """ Given points p1, p2 in R^2 it returns the norm of the vector connecting them.  """
    dist = ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
    return dist


def angle_between_three_points(p1, p2, p3):
    """ Given points p1, p2, p3 returns the angle between them.  """
    point_a = p1 - p2
    point_b = p3 - p2

    ang_a = np.arctan2(*point_a[::-1])
    ang_b = np.arctan2(*point_b[::-1])

    return np.rad2deg((ang_a - ang_b) % (2 * np.pi))


def pickle_data(data, filename):
    """ save the metrics on file """
    with open(filename, 'wb') as out:
        pickle.dump(data, out)


def unpickle_data(filename):
    """ load the metrics from a file """
    with open(filename, 'rb') as handle:
        obj = pickle.load(handle)
    return obj


def save_txt(text, file):
    with open(file, "w") as f:
        f.write(text)


def distance_point_segment(ps1, ps2, external_p):
    """Returns the distance between segment and point. Don't ask why."""
    x3, y3 = external_p
    x1, y1 = ps1
    x2, y2 = ps2

    px = x2-x1
    py = y2-y1

    norm = px*px + py*py

    u =  ((x3 - x1) * px + (y3 - y1) * py) / float(norm)

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    x = x1 + u * px
    y = y1 + u * py

    dx = x - x3
    dy = y - y3

    dist = (dx*dx + dy*dy)**.5

    return dist


def is_segments_intersect(A, B, C, D):
    """ Return true if line segments AB and CD intersect """
    segment1 = LineString([A, B])
    segment2 = LineString([C, D])
    point = segment1.intersection(segment2)

    return not point.is_empty


# ------------------ Event (Traffic) Generator ----------------------
class EventGenerator:

    def __init__(self, simulator):
        """
        :param simulator: the main simulator object
        """
        self.simulator = simulator
        self.rnd_drones = np.random.RandomState(self.simulator.seed)
        # for now no random on number of event generated
        # self.rnd_event = np.random.RandomState(self.simulator.seed)

    def handle_events_generation(self, cur_step : int, drones : list):
        """
        at fixed time randomly select a drone from the list and sample on it a packet/event.

        :param cur_step: the current step of the simulation to decide whenever sample an event or not
        :param drones: the drones where to sample the event
        :return: nothing
        """
        if cur_step > 0 and cur_step % self.simulator.event_generation_delay == 0:  # if it's time to generate a new packet
            print(self.simulator.event_generation_delay, cur_step, cur_step % self.simulator.event_generation_delay)
            # drone that will receive the packet:
            drone_index = self.rnd_drones.randint(0, len(drones))
            drone = drones[drone_index]
            drone.feel_event(cur_step)

# ------------------ Path manager ----------------------
class PathManager:

    def __init__(self, json_file: str, seed: int):
        """ json file to read for take the paths of drones
            We assume json_file + seed + .json
        """
        self.json_file = json_file.replace(".json", "") + str(seed) + ".json"
        self.path_dict = json_to_paths(self.json_file)

    def path(self, drone_id):
        """ takes the drone id and
            returns a path (list of tuple)
            for it.

            Notice that: the path can last
            less or more than the simulation.
            In the first case the path should be repeated.
        """
        if config.HANDCRAFTED_PATH:
            return self.__demo_path(drone_id)
        else:
            return self.path_dict[drone_id]

    def __demo_path(self, drone_id):
        """ Add handcrafted torus here.  """
        tmp_path = {0: [(750, 750), (760, 750), (750, 750), (760, 750), (770, 750)],
                    1: [(1280, 80),  (760-300, 750+300), (760+300, 750+300), (760+300, 750-300), (760-300, 750-300),  (0, 1500)],
                    2: [(1320, 120), (760-300, 750+300), (760+300, 750+300), (760+300, 750-300), (760-300, 750-300), (0, 1500)],
                    3: [(1400, 160), (760-300, 750+300), (760+300, 750+300), (760+300, 750-300), (760-300, 750-300), (0, 1500)],
                    4: [(1500, 200), (760-300, 750+300), (760+300, 750+300), (760+300, 750-300), (760-300, 750-300),  (0, 1500)]}
        return tmp_path[drone_id]


def json_to_paths(json_file_path):
    """ load the tour for drones
        and return a dictionary {drone_id : list of waypoint}

        e.g.,
        accept json that contains:
        {"drones": [{"index": "0", "tour": ["(1500, 0)", "(1637, 172)", ...
                    (1500, 0)"]}, {"index": "1", "tour": ["(1500, 0)",

        TOURS = {
            0 : [(0,0), (2000,2000), (1500, 1500), (200, 2000)],
            1 : [(0,0), (2000, 200), (200, 2000), (1500, 1500)]
        }
    """
    out_data = {}
    with open(json_file_path, 'r') as in_file:
        data = json.load(in_file)
        for drone_data in data["drones"]:
            drone_index = int(drone_data["index"])
            drone_path = []
            for waypoint in drone_data["tour"]:
                drone_path.append(make_tuple(waypoint))
            out_data[drone_index] = drone_path
    return out_data


class LimitedList:
    """ Time window """
    def __init__(self, threshold=None):
        self.llist = []
        self.threshold = threshold

    def append(self, el):
        if self.threshold and self.threshold < len(self.llist) + 1:
            self.llist = self.llist[1:]
        self.llist.append(el)

    def __len__(self):
        return len(self.llist)

    def __getitem__(self, index):
        return self.llist[index]


def make_path(fname):
    path = pathlib.Path(fname)
    path.parent.mkdir(parents=True, exist_ok=True)


def plot_X(X, plt_title, plt_path, window_size=30, is_avg=True):
    if len(X) >= window_size:
        df = pd.Series(X)
        scatter_print = X[window_size:]
        to_plot_data = df.rolling(window_size).mean()[window_size:]

        plt.clf()
        plt.plot(range(len(scatter_print)), to_plot_data, label="Moving Average-" + str(window_size))
        if is_avg:
            plt.plot(range(len(scatter_print)), [np.average(scatter_print)] * len(scatter_print), label="avg")

        plt.legend()
        plt.title(plt_title)
        plt.savefig(plt_path)
        plt.clf()


""" This class handle the return to depot for
    the drones, such that they return to the depot in a coordinated fashion
    currently is based on channel -> in future can also handle cluster head/waypoints
"""

class PathToDepot():

    def __init__(self, x_position, simulator):
        """ for now just a middle channel in the area used by all the drones """
        self.x_position = x_position
        self.simulator = simulator

    def next_target(self, drone_pos):
        """ based on the drone position return the next target:
            |-> channel position or cluster head position
            |-> the depot if the drones are already in the channel or have overpass the cluster head
        """
        # only channel mode
        if abs(drone_pos[
                   0] - self.x_position) < 1:  # the drone is already on the channel with an error of 1 meter
            return self.simulator.depot_coordinates
        else:
            return self.x_position, drone_pos[1]  # the closest point to the channel


def measure_scaler(measure, dom_start, dom_target):
    """ Scales the measure value in the start domain [Type, min, max], in the target domain. """
    return (measure - dom_start[1]) / (dom_start[2] - dom_start[1]) * (dom_target[2] - dom_target[1]) + dom_target[1]

# -------------------- all cells computation ---------------------#

class TraversedCells:

    @staticmethod
    def cells_in_travel(size_cell, width_area, start, end):
        """ return the cell number in which the pos (x, y) lay """

        start_cell, coords_cell_start = TraversedCells.coord_to_cell(size_cell, width_area, start[0], start[1])  # int, lower left coordinates
        end_cell, coords_cell_end = TraversedCells.coord_to_cell(size_cell, width_area, end[0], end[1])

        out_cells = []

        if coords_cell_end[1] == coords_cell_start[1]:  # vertical alignment
            min_x = min(coords_cell_start[0], coords_cell_end[0])
            max_x = max(coords_cell_start[0], coords_cell_end[0])
            for x_ in range(min_x, max_x + 1):
                out_cells.append((x_, coords_cell_end[1]))
            return out_cells

        if coords_cell_end[0] == coords_cell_start[0]:  # horizontal alignment
            min_y = min(coords_cell_start[1], coords_cell_end[1])
            max_y = max(coords_cell_start[1], coords_cell_end[1])
            for y_ in range(min_y, max_y + 1):
                out_cells.append((coords_cell_end[0], y_))
            return out_cells

        # Diagonal line
        # Boundaries of the rectangle
        min_x, max_x = min(coords_cell_start[0], coords_cell_end[0]), max(coords_cell_start[0], coords_cell_end[0])
        min_y, max_y = min(coords_cell_start[1], coords_cell_end[1]), max(coords_cell_start[1], coords_cell_end[1])

        # All the cells of the rectangle, indices
        coords_index = [(i, j) for i in range(min_x, max_x+1) for j in range(min_y, max_y+1)]
        for cell in coords_index:

            ll = cell[0]*size_cell, cell[1]*size_cell
            lr = cell[0]*size_cell + size_cell, cell[1]*size_cell
            ul = cell[0]*size_cell, cell[1]*size_cell + size_cell
            ur = cell[0]*size_cell + size_cell, cell[1]*size_cell + size_cell

            if TraversedCells.intersect_quad(start, end, ll, lr, ul, ur):
                out_cells.append(cell)

        return out_cells  # list of lower-lefts

    @staticmethod
    def intersect_quad(start, end, ll, lr, ul, ur):

        return (TraversedCells.intersect_segments(start, end, ll, lr)
                or TraversedCells.intersect_segments(start, end, ul, ur)
                or TraversedCells.intersect_segments(start, end, ul, ll)
                or TraversedCells.intersect_segments(start, end, lr, ur))

    @staticmethod
    def intersect_segments(start1:tuple, end1:tuple, start2:tuple, end2:tuple):
        if end1 == start2:
            return True
        if end2 == start1:
            return True
        if start2 == start1:
            return True
        if end2 == end1:
            return True

        a = np.asarray(end1) - np.asarray(start1)  # direction of line a
        b = np.asarray(start2) - np.asarray(end2)  # direction of line b, reversed
        d = np.asarray(start2) - np.asarray(start1)  # right-hand side
        det = a[0] * b[1] - a[1] * b[0]

        if det == 0:
            return False

        t = (a[0] * d[1] - a[1] * d[0]) / det
        return 0 <= t <= 1

    @staticmethod
    def all_centers(widht_area, height_area, size_cell):
        """ return all cell along their centers """
        all_cells_and_centers = []
        for x in range(0, widht_area, size_cell):
            for y in range(0, height_area, size_cell):
                all_cells_and_centers.append(
                    (TraversedCells.coord_to_cell(size_cell, widht_area, x, y),
                        (x + (size_cell/2.0), y + (size_cell/2.0)))
                )
        return all_cells_and_centers

    @staticmethod
    def coord_to_cell(size_cell, width_area, x_pos, y_pos):
        """ return the cell number in which the pos (x"abs", y"abs") lay """
        x_cell_coords = int(x_pos / size_cell)
        y_cell_coords = int(y_pos / size_cell)
        return TraversedCells.cell_coord_to_cell_number(size_cell, width_area, x_cell_coords,
                                                        y_cell_coords), (x_cell_coords, y_cell_coords)

    @staticmethod
    def cell_coord_to_cell_number(size_cell, width_area, x_cell_coords, y_cell_coords):
        """ return the number o the cells given the indexes """

        x_cells = np.ceil(width_area / size_cell)  # numero di celle su X
        return x_cell_coords + (x_cells * y_cell_coords)

