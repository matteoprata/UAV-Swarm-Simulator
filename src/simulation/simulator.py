
from src.world_entities.environment import Environment
from src.world_entities.base_station import BaseStation
from src.world_entities.drone import Drone

from src.utilities.utilities import PathManager, current_date, euclidean_distance
import src.utilities.config as config
from src.drawing import pp_draw

import numpy as np
import time


class Simulator:

    def __init__(self,
                 sim_seed=config.SIM_SEED,
                 ts_duration_sec=config.SIM_TS_DURATION,
                 sim_duration_ts=config.SIM_DURATION,
                 n_drones=config.N_DRONES,
                 env_width_meters=config.ENV_WIDTH,
                 env_height_meters=config.ENV_HEIGHT,
                 drone_speed=config.DRONE_SPEED,
                 drone_max_battery=config.DRONE_MAX_ENERGY,
                 drone_max_buffer=config.DRONE_MAX_BUFFER_SIZE,
                 drone_com_range_meters=config.DRONE_COM_RANGE,
                 drone_sen_range_meters=config.DRONE_SENSING_RANGE,
                 drone_radar_range_meters=config.DRONE_RADAR_RADIUS,
                 drone_coo=config.INITIAL_DRONE_COORDS,
                 bs_com_range_meters=config.BASE_STATION_COM_RANGE,
                 bs_coords=config.BASE_STATION_COORDS,
                 n_obstacles=config.N_OBSTACLES,
                 n_grid_cells=config.N_GRID_CELLS,
                 target_coods=config.TARGETS_COORDS
                 ):

        self.cur_step = 0
        self.sim_seed = sim_seed
        self.ts_duration_sec = ts_duration_sec
        self.sim_duration_ts = sim_duration_ts
        self.env_width_meters, self.env_height_meters = env_width_meters, env_height_meters
        self.n_drones = n_drones
        self.n_obstacles = n_obstacles
        self.grid_cell_size = 0 if n_grid_cells <= 0 else int(self.env_width_meters / n_grid_cells)

        # if this coo is not none, then the drones are self driven
        self.drone_coo = drone_coo
        self.selected_drone = None

        self.drone_speed_meters_sec = drone_speed
        self.drone_max_battery = drone_max_battery
        self.drone_max_buffer = drone_max_buffer
        self.drone_com_range_meters = drone_com_range_meters
        self.drone_sen_range_meters = drone_sen_range_meters
        self.drone_radar_range_meters = drone_radar_range_meters
        self.bs_com_range_meters = bs_com_range_meters
        self.bs_coords = bs_coords
        self.target_coods = target_coods

        # create the world entites
        self.__set_randomness()
        self.__create_world_entities()
        self.__setup_plotting()

    def simulation_duration_sec(self):
        return self.sim_duration_ts * self.ts_duration_sec

    def simulation_name(self):
        return "seed{}-ndrones{}-date{}".format(self.sim_seed, self.n_drones, current_date())

    def is_free_movement(self):
        return self.drone_coo is not None

    def detect_key_pressed(self, key_pressed):
        """ Moves the drones freely. """

        if key_pressed in ['a', 'A']:  # decrease angle
            self.selected_drone.angle -= config.DRONE_ANGLE_INCREMENT
            self.selected_drone.angle = self.selected_drone.angle % 360

        elif key_pressed in ['d', 'D']:  # increase angle
            self.selected_drone.angle += config.DRONE_ANGLE_INCREMENT
            self.selected_drone.angle = self.selected_drone.angle % 360

        elif key_pressed in ['w', 'W']:  # increase speed
            self.selected_drone.speed += config.DRONE_SPEED_INCREMENT

        elif key_pressed in ['s', 'S']:  # decrease speed
            self.selected_drone.speed -= config.DRONE_SPEED_INCREMENT

    def detect_drone_click(self, position):
        """ Handles drones selection in the simulation. """
        click_coords_to_map = (self.environment.width/config.DRAW_SIZE*position[0], self.environment.height/config.DRAW_SIZE*(config.DRAW_SIZE-position[1]))
        entities_distance = [euclidean_distance(drone.coords, click_coords_to_map) for drone in self.environment.drones]
        clicked_drone = self.environment.drones[np.argmin(entities_distance)] # potentially clicked drone

        TOLERATED_CLICK_DISTANCE = 40

        closest_drone_coords = clicked_drone.coords
        dron_coords_to_screen = (closest_drone_coords[0]*config.DRAW_SIZE/self.environment.width, config.DRAW_SIZE - (closest_drone_coords[1]*config.DRAW_SIZE/self.environment.width))

        if euclidean_distance(dron_coords_to_screen, position) < TOLERATED_CLICK_DISTANCE:
            # DRONE WAS CLICKED HANDLE NOW
            self.on_drone_click(clicked_drone)

    def on_drone_click(self, clicked_drone):
        """ Defines the behaviour following a click on a drone. """
        self.selected_drone = clicked_drone

    def __setup_plotting(self):
        if config.PLOT_SIM or config.SAVE_PLOT:
            self.draw_manager = pp_draw.PathPlanningDrawer(self.environment, self, borders=True)

    def __set_randomness(self):
        """ Set the random generators. """
        self.rnd_env = np.random.RandomState(self.sim_seed)
        self.rnd_event = np.random.RandomState(self.sim_seed)

    def __create_world_entities(self):
        """ Creates the world entities. """

        if self.drone_coo is None:
            self.path_manager = PathManager(config.FIXED_TOURS_DIR + "RANDOM_missions", self.sim_seed)

        self.environment = Environment(self.env_width_meters, self.env_height_meters, self)

        self.environment.spawn_obstacles()
        self.environment.spawn_targets(self.target_coods)

        base_stations = [BaseStation(self.bs_coords, self.bs_com_range_meters, self)]

        drones = [Drone(identifier=i,
                        path=[self.drone_coo] if self.is_free_movement() else self.path_manager.path(i),
                        bs=base_stations[0],
                        angle=0,
                        speed=0 if self.is_free_movement() else self.drone_speed_meters_sec,
                        com_range=self.drone_com_range_meters,
                        sensing_range=self.drone_sen_range_meters,
                        radar_range=self.drone_radar_range_meters,
                        max_buffer=self.drone_max_buffer,
                        max_battery=self.drone_max_battery,
                        simulator=self

                        ) for i in range(self.n_drones)]

        self.selected_drone = drones[0]
        self.environment.add_base_station(base_stations)
        self.environment.add_drones(drones)

    def __plot(self, cur_step):
        """ Plot the simulation """

        if cur_step % config.SKIP_SIM_STEP != 0:
            return

        if config.WAIT_SIM_STEP > 0:
            time.sleep(config.WAIT_SIM_STEP)

        self.draw_manager.grid_plot()
        self.draw_manager.borders_plot()

        for drone in self.environment.drones:
            self.draw_manager.draw_drone(drone, cur_step)

        for base_station in self.environment.base_station:
            self.draw_manager.draw_depot(base_station)

        for event in self.environment.get_valid_events(cur_step):
            self.draw_manager.draw_event(event)

        self.draw_manager.draw_simulation_info(cur_step=cur_step, max_steps=self.sim_duration_ts)
        self.draw_manager.draw_obstacles()
        self.draw_manager.draw_target(self.target_coods)
        self.draw_manager.update(save=config.SAVE_PLOT, filename=self.simulation_name() + str(cur_step) + ".png")

    def run(self):
        """ The method starts the simulation. """

        for cur_step in range(self.sim_duration_ts):
            self.cur_step = cur_step

            for drone in self.environment.drones:
                self.environment.detect_collision(drone)
                drone.move()

            if config.SAVE_PLOT or config.PLOT_SIM:
                self.__plot(cur_step)

    def print_metrics(self):
        pass

    def close(self):
        pass
