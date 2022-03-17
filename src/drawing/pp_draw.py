from src.drawing import stddraw
from src.world_entities.environment import Environment
from src.utilities import config, utilities
from collections import defaultdict
import numpy as np


class PathPlanningDrawer:

    def __init__(self, env : Environment, simulator, borders=False, padding=25):
        """ init the path plannind drawer """
        self.width =  env.width
        self.height =  env.height
        self.borders = borders
        self.simulator = simulator
        stddraw.setXscale(0 - padding, self.width + padding)
        stddraw.setYscale(0 - padding, self.height + padding)

        if self.borders:
            self.borders_plot()

        self.keep_indictor = defaultdict(list)  # list of couples (time stamp, drone)

    def __channel_to_depot(self):
        stddraw.setPenColor(c=stddraw.LIGHT_GRAY)
        stddraw.setPenRadius(0.0025)
        stddraw.line(self.width/2, self.height, self.width/2, 0)
        self.__reset_pen()

    def save(self, filename):
        """ save the current plot """
        stddraw.save(filename)

    def borders_plot(self):
        stddraw.setPenColor(c=stddraw.RED)
        stddraw.setPenRadius(0.002)
        stddraw.line(0, 0, 0, self.width)
        stddraw.line(0, 0, self.height, 0)
        stddraw.line(0, self.width, self.height, self.width)
        stddraw.line(self.height, 0, self.height, self.width)
        self.__reset_pen()

    def grid_plot(self):
        """ Plots the tassellation of the area."""
        if not self.simulator.grid_cell_size > 0:
            return

        # stddraw.setPenColor(c=stddraw.RED)
        # stddraw.setPenRadius(0.0025)

        stddraw.setPenColor(c=stddraw.VERY_LIGHT_GRAY)
        stddraw.setPenRadius(0.002)

        for i in range(self.simulator.grid_cell_size, self.width, self.simulator.grid_cell_size):
            stddraw.line(i, 0, i, self.height)

        for j in range(self.simulator.grid_cell_size, self.height, self.simulator.grid_cell_size):
            stddraw.line(0, j, self.width, j)
        self.__reset_pen()

    def __reset_pen(self):
        stddraw.setPenColor(c=stddraw.BLACK)
        stddraw.setPenRadius(0.0055)

    def midpoint_line(self, startx, starty, endx, endy):
        return np.asarray([(startx + endx)/2, (starty + endy)/2])

    def draw_obstacles(self):
        """ Spawns random obstacle segments in the map. """
        stddraw.setPenColor(c=stddraw.RED)
        stddraw.setPenRadius(0.002)

        for i in range(self.simulator.n_obstacles):
            # generation is done only at the beginning
            startx, starty, endx, endy = self.simulator.environment.obstacles[i]
            stddraw.line(startx, starty, endx, endy)

        self.__reset_pen()

    def draw_drone(self, drone, cur_step):
        coords = drone.coords
        if drone.buffer_length() > 0:  # change color when find a packet
            stddraw.setPenColor(c=stddraw.GREEN)
        else:     
            stddraw.setPenColor(c=stddraw.BLACK)
        stddraw.setPenRadius(0.0055)
        stddraw.point(coords[0], coords[1])

        self.__draw_drone_info(drone, cur_step)
        self.__draw_communication_range(drone)
        self.__draw_sensing_range(drone)

        # the radar distance
        self.__draw_distance_radar(coords[0], coords[1], drone.radar_range)

        self.__reset_pen()

        if config.PLOT_TRAJECTORY_NEXT_TARGET and not self.simulator.is_free_movement():
            self.__draw_next_target(drone.coords, drone.next_target())

    def __validate_rew(self, drone, cur_step):
        coords = drone.coords

        stddraw.setPenColor(c=stddraw.RED)

        # reward: relay found when doing broadcast
        if drone.routing_algorithm.draw_rew_broad is not None:
            r = drone.routing_algorithm.draw_rew_broad
            self.keep_indictor["r.bro"].append((cur_step, drone, r))
            drone.routing_algorithm.draw_rew_broad = None

        # reward: packet expired
        if drone.routing_algorithm.draw_rew_exp is not None:
            r = drone.routing_algorithm.draw_rew_exp
            self.keep_indictor["r.exp"].append((cur_step, drone, r))
            drone.routing_algorithm.draw_rew_exp = None

        # reward: packet delivered to depot
        if drone.routing_algorithm.draw_rew_dep is not None:
            r = drone.routing_algorithm.draw_rew_dep
            self.keep_indictor["r.dep"].append((cur_step, drone, r))
            drone.routing_algorithm.draw_rew_dep = None

        # reward: every delta did move
        if drone.routing_algorithm.draw_rew_area_move is not None:
            r = drone.routing_algorithm.draw_rew_area_move
            self.keep_indictor["r.dmo"].append((cur_step, drone, r))
            drone.routing_algorithm.draw_rew_area_move = None

        # reward: every delta did broad
        if drone.routing_algorithm.draw_rew_area_broad is not None:
            r = drone.routing_algorithm.draw_rew_area_broad
            self.keep_indictor["r.dbr"].append((cur_step, drone, r))
            drone.routing_algorithm.draw_rew_area_broad = None

        for k in self.keep_indictor:
            for ts, drone_in, r in self.keep_indictor[k]:
                if drone == drone_in:
                    # red for all rewards except those evaluated every delta
                    if k in ["r.dmo", "r.dbr"]: stddraw.setPenColor(c=stddraw.MAGENTA)
                    else: stddraw.setPenColor(c=stddraw.RED)

                    stddraw.text(coords[0] + 30, coords[1] + 30, k + " " + str(round(r, 3)))

                    if cur_step - ts >= 100:
                        self.keep_indictor[k].remove((ts, drone_in, r))

    def update(self, rate=1, 
                save=False, show=True,
                filename=None):
        """ update the draw """

        if show:
            stddraw.show(self.simulator, rate)
        if save:
            assert(filename is not None)
            self.save(filename)
        stddraw.clear()
        
    def draw_event(self, event):
        coords = event.coords
        stddraw.setPenRadius(0.0055)
        stddraw.setPenColor(c=stddraw.RED)
        stddraw.point(coords[0], coords[1])
        stddraw.setPenColor()
        self.__reset_pen()

    def draw_vector(self, pos, vector):
        stddraw.setPenRadius(0.0500)
        stddraw.line(pos[0], pos[1], vector[0], vector[1])

    def draw_depot(self, depot):
        coords = depot.coords
        stddraw.setPenRadius(0.0100)
        stddraw.setPenColor(c=stddraw.BLUE)
        stddraw.point(coords[0], coords[1])
        self.__draw_communication_range(depot)
        self.__reset_pen()

        # draw the buffer size
        stddraw.setPenRadius(0.0125)
        stddraw.setPenColor(c=stddraw.BLACK)
        stddraw.text(depot.coords[0], depot.coords[1]+100, "pk: " + str(len(depot.buffer)))

    def draw_target(self, target_coords):
        for i, (startx, starty) in enumerate(target_coords):
            stddraw.setPenColor(c=stddraw.BOOK_LIGHT_BLUE)
            stddraw.filledSquare(startx, starty, 10)
            stddraw.text(startx, starty + 25, "tar id:{}".format(i))

        self.__reset_pen()

    def __draw_sensing_range(self, body):
        stddraw.setPenRadius(0.0015)
        stddraw.setPenColor(c=stddraw.RED)
        stddraw.circle(body.coords[0], body.coords[1], body.sensing_range)
        stddraw.setPenColor(c=stddraw.BLACK)

    def __draw_communication_range(self, body):
        stddraw.setPenRadius(0.0015)
        stddraw.setPenColor(c=stddraw.BLUE)
        stddraw.circle(body.coords[0], body.coords[1], body.com_range)
        stddraw.setPenColor(c=stddraw.BLACK)

    def __draw_distance_radar(self, x, y, radar_radius):
        stddraw.setPenRadius(0.0015)
        stddraw.setPenColor(c=stddraw.RED)
        stddraw.circle(x, y, radar_radius)
        stddraw.setPenColor(c=stddraw.BLACK)

    def draw_blocks(self, drone_coo, target, size_cell, cells):

        for coord in cells:
            stddraw.setPenColor(c=stddraw.CYAN)
            stddraw.filledRectangle(coord[0] * size_cell, coord[1] * size_cell, size_cell, size_cell)
            self.__reset_pen()

    def __draw_next_target(self, drone_coo, target):

        stddraw.setPenRadius(0.0055)
        stddraw.setPenColor(c=stddraw.BLUE)
        stddraw.point(target[0], target[1])
        stddraw.setPenColor()
        self.__reset_pen()

        stddraw.setPenColor(c=stddraw.BLUE)
        stddraw.setPenRadius(0.0025)
        stddraw.line(drone_coo[0], drone_coo[1], target[0], target[1])
        self.__reset_pen()

    def __draw_drone_info(self, drone, cur_step):
        stddraw.setPenRadius(0.0125)
        stddraw.setPenColor(c=stddraw.BLACK)

        # index
        stddraw.text(drone.coords[0], drone.coords[1] + (drone.com_range / 2.0), "id: " + str(drone.identifier))

    def draw_simulation_info(self, cur_step, max_steps):
        TEXT_LEFT = 60
        TEXT_TOP = 30

        stddraw.text(TEXT_LEFT + 20, self.height - TEXT_TOP, str(cur_step) + "/" + str(max_steps))
        #if self.simulator.is_free_movement():
        stddraw.text(TEXT_LEFT + 20, self.height - TEXT_TOP*2, "drone: " + str(self.simulator.selected_drone.identifier))
        stddraw.text(TEXT_LEFT + 20, self.height - TEXT_TOP*3, "speed: " + str(self.simulator.selected_drone.speed))
        stddraw.text(TEXT_LEFT + 20, self.height - TEXT_TOP*4, "angle: " + str(self.simulator.selected_drone.angle))
        self.__reset_pen()
