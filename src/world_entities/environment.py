
from src.utilities.utilities import log, is_segments_intersect, distance_point_segment, TraversedCells
from src.world_entities.target import Target

import numpy as np


class Environment:
    """ The environment is an entity that represents the area of interest."""

    def __init__(self, width, height, simulator):

        self.simulator = simulator
        self.width = width
        self.height = height

        # set events, set obstacles
        self.events: list = []  # even expired ones
        self.obstacles = []
        self.targets = []

    def add_drones(self, drones: list):
        """ add a list of drones in the env """
        log("Added {} drones to the environment.".format(len(drones)))
        self.drones = drones

    def add_base_station(self, base_stations: list):
        """ add depots in the env """
        log("Added {} base stations in the environment.".format(len(base_stations)))
        self.base_station = base_stations

    def get_expired_events(self, current_ts):
        return [e for e in self.events if e.is_expired(current_ts)]

    def get_valid_events(self, current_ts):
        return [e for e in self.events if not e.is_expired(current_ts)]

    def query_drone_sensing(self, drone):
        """ Returns a list of valid events nj
        that this drone is able to sense. """
        pass

    def get_current_cell(self, drone):
        drone_coods = drone.coords
        cell_index = TraversedCells.coord_to_cell(size_cell=self.simulator.grid_cell_size,
                                                  width_area=self.width,
                                                  x_pos=drone_coods[0],
                                                  y_pos=drone_coods[1])
        return cell_index

    def spawn_obstacles(self, orthogonal_obs=False):
        """ Appends obstacles in the environment """
        for i in range(self.simulator.n_obstacles):
            startx, starty = self.simulator.rnd_env.randint(0, self.width), self.simulator.rnd_env.randint(0, self.height)
            length = self.simulator.rnd_env.randint(100, 300)
            angle = self.simulator.rnd_env.randint(0, 359) if not orthogonal_obs else self.simulator.rnd_env.choice([0, 90, 180, 270], 1)[0]

            endx, endy = startx + length * np.cos(np.radians(angle)), starty + length * np.sin(np.radians(angle))
            obstacle = (startx, starty, endx, endy)
            self.obstacles.append(obstacle)

    def spawn_targets(self, target_coord):
        """ Spawns target that have infinite tolerance. """
        for i, coords in enumerate(target_coord):
            self.targets.append(Target(i, coords, self.simulator.simulation_duration_sec(), self.simulator))

    def detect_collision(self, drone):
        """ Detects a collision happened in the previous step. """
        if len(self.obstacles) > 0:

            # drone - obstacle collision
            distance_obstacles = self.distance_obstacles(drone)
            distance_travelled = drone.speed * self.simulator.ts_duration_sec
            critic_obstacles = np.array(self.obstacles)[distance_obstacles <= distance_travelled]  # obstacles that could be crossed in a time step

            for critical_segment in critic_obstacles:
                p1, p2 = (critical_segment[0], critical_segment[1]), (critical_segment[2], critical_segment[3])
                p3, p4 = drone.previous_coords, drone.coords

                if is_segments_intersect(p1, p2, p3, p4) or distance_point_segment(p1, p2, p4) < 1:
                    # COLLISION HAPPENED DO SOMETHING
                    self.__handle_collision(drone)
                    return

        # # drone - drone collision to fix
        # if drone.coords != self.simulator.drone_coo:
        #     u1p1, u1p2 = drone.previous_coords, drone.coords
        #
        #     path_u1p1 = [0, u1p1[0], u1p1[1]]
        #     path_u1p2 = [self.simulator.ts_duration_sec,
        #                  self.simulator.ts_duration_sec * drone.speed + u1p1[0],
        #                  self.simulator.ts_duration_sec * drone.speed + u1p1[1]]
        #
        #     for other_drone in self.drones:
        #         if drone.identifier > other_drone.identifier:
        #             u2p1, u2p2 = other_drone.previous_coords, other_drone.coords
        #
        #             path_u2p1 = [0, u2p1[0], u2p1[1]]
        #             path_u2p2 = [self.simulator.ts_duration_sec,
        #                          self.simulator.ts_duration_sec * drone.speed + u2p1[0],
        #                          self.simulator.ts_duration_sec * drone.speed + u2p1[1]]
        #
        #             if is_segments_intersect(path_u1p1, path_u1p2, path_u2p1, path_u2p2) \
        #                     or euclidean_distance(u1p2, u2p2) < 1:
        #                 self.__handle_collision(drone)
        #                 self.__handle_collision(other_drone)
        #                 return

    def distance_obstacles(self, drone):
        """ Returns the distance for all the obstacles. """

        distance_obstacles = np.array([-1] * self.simulator.n_obstacles)
        p3 = np.array(drone.coords)
        for en, ob in enumerate(self.obstacles):
            p1, p2 = np.array([ob[0], ob[1]]), np.array([ob[2], ob[3]])
            distance_obstacles[en] = distance_point_segment(p1, p2, p3)
        return distance_obstacles

    def __handle_collision(self, drone):
        """ Takes countermeasure when drone collides. """
        drone.coords = drone.path[0]
