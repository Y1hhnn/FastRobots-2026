#!/usr/bin/env python3
import time
import numpy as np
import math
from copy import deepcopy
from robot import VirtualRobot
from utils import get_max, setup_logging

LOG = setup_logging("localization.log")


class Mapper():
    """A class to perform various mapping-related processing required for grid localization
    """

    def __init__(self, robot):

        self.robot = robot

        # Map limits
        self.MIN_X = robot.config_params["mapper"]["min_x"]
        self.MAX_X = robot.config_params["mapper"]["max_x"]
        self.MIN_Y = robot.config_params["mapper"]["min_y"]
        self.MAX_Y = robot.config_params["mapper"]["max_y"]
        self.MIN_A = robot.config_params["mapper"]["min_a"]
        self.MAX_A = robot.config_params["mapper"]["max_a"]

        # Cell dimensions
        self.CELL_SIZE_X = robot.config_params["mapper"]["cell_size_x"]
        self.HALF_CELL_SIZE_X = self.CELL_SIZE_X / 2.0
        self.CELL_SIZE_Y = robot.config_params["mapper"]["cell_size_y"]
        self.HALF_CELL_SIZE_Y = self.CELL_SIZE_Y / 2.0
        self.CELL_SIZE_A = robot.config_params["mapper"]["cell_size_a"]
        self.HALF_CELL_SIZE_A = self.CELL_SIZE_A / 2.0

        # Cell Maxima
        self.MAX_CELLS_X = robot.config_params["mapper"]["max_cells_x"]
        self.MAX_CELLS_Y = robot.config_params["mapper"]["max_cells_y"]
        self.MAX_CELLS_A = robot.config_params["mapper"]["max_cells_a"]

        # Center cells
        self.CENTER_CELL_X = int(self.MAX_CELLS_X/2)
        self.CENTER_CELL_Y = int(self.MAX_CELLS_Y/2)
        self.CENTER_CELL_A = int(self.MAX_CELLS_A/2)

        # Ray tracing parameters
        self.OBS_PER_CELL = int(
            robot.config_params["mapper"]["observations_count"])
        self.RAY_TRACING_ANGLE_INCREMENT = 360/self.OBS_PER_CELL
        self.RAY_LENGTH = robot.config_params["mapper"]["ray_tracing_length"]

        # Map Cells
        self.cells = np.zeros((self.MAX_CELLS_X,
                               self.MAX_CELLS_Y,
                               self.MAX_CELLS_A))
        # Map rays for each cell
        self.obs_views = np.zeros((self.MAX_CELLS_X,
                                   self.MAX_CELLS_Y,
                                   self.MAX_CELLS_A,
                                   self.OBS_PER_CELL
                                   ))
        # Ray Intersection points based on the map
        self.obs_points_x = np.zeros((self.MAX_CELLS_X,
                                      self.MAX_CELLS_Y,
                                      self.MAX_CELLS_A,
                                      self.OBS_PER_CELL
                                      ))
        self.obs_points_y = np.zeros((self.MAX_CELLS_X,
                                      self.MAX_CELLS_Y,
                                      self.MAX_CELLS_A,
                                      self.OBS_PER_CELL
                                      ))

        # x, y and a indices in real world coordinates for each cell index
        self.x_values = np.zeros(
            (self.MAX_CELLS_X, self.MAX_CELLS_Y, self.MAX_CELLS_A))
        self.y_values = np.zeros(
            (self.MAX_CELLS_X, self.MAX_CELLS_Y, self.MAX_CELLS_A))
        self.a_values = np.zeros(
            (self.MAX_CELLS_X, self.MAX_CELLS_Y, self.MAX_CELLS_A))

        self.lines = [np.array([e[0] for e in robot.config_params["map_lines"]]),
                      np.array([e[1] for e in robot.config_params["map_lines"]])]

        self.populate_views()

    # Ref: https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees
    # https://stackoverflow.com/questions/36717163/python-numpy-radians-to-degrees-in-0-360

    def normalize_angle(self, a):
        new_a = a
        while (new_a < -180):
            new_a = new_a + 360
        while (new_a >= 180):
            new_a = new_a - 360
        return new_a

    # Return the continuous world coordinates (x,y,z) [in (m,m,deg)] of the center of the grid cell (cx, cy, cz)
    def from_map(self, cx, cy, ca):
        x = cx*self.CELL_SIZE_X + self.MIN_X + self.HALF_CELL_SIZE_X
        y = cy*self.CELL_SIZE_Y + self.MIN_Y + self.HALF_CELL_SIZE_Y
        a = ca*self.CELL_SIZE_A + self.MIN_A + self.HALF_CELL_SIZE_A
        return x, y, self.normalize_angle(a)

    # Return the grid cell index (cx,cy,cz) of the point (x, y, a) [in (m,m,deg)] in the continuous world frame
    def to_map(self, x, y, a):
        a = self.normalize_angle(a)
        cx = (x/self.CELL_SIZE_X) + self.CENTER_CELL_X
        cy = (y/self.CELL_SIZE_Y) + self.CENTER_CELL_Y
        ca = (a/self.CELL_SIZE_A) + self.CENTER_CELL_A
        return int(cx), int(cy), int(ca)

    def cross_product_single_multiple(self, v1, v2):
        return v1[0]*v2[:, 1] - v1[1]*v2[:, 0]

    def cross_product_multiple(self, v1, v2):
        return v1[:, 0]*v2[:, 1] - v1[:, 1]*v2[:, 0]

    def cross_product_multiple_single(self, v1, v2):
        return v1[:, 0]*v2[1] - v1[:, 1]*v2[0]

    def get_intersection(self, ray, pose):
        try:
            with np.errstate(divide='ignore'):

                denom = self.cross_product_single_multiple(ray[1] - ray[0],
                                                           self.lines[1] - self.lines[0])

                t = self.cross_product_multiple(self.lines[0] - ray[0],
                                                self.lines[1] - self.lines[0])
                t = t / denom

                u = self.cross_product_multiple_single(self.lines[0] - ray[0],
                                                       ray[1] - ray[0]) / denom

                t[(t < 0) | (t > 1)] = np.nan
                u[(u < 0) | (u > 1)] = np.nan

                tt = np.copy(t)
                tt[np.isnan(t) | np.isnan(u)] = np.nan

                uu = np.copy(u)
                uu[np.isnan(t) | np.isnan(u)] = np.nan

                intersections_tt = ray[0] + tt[:, np.newaxis]*(ray[1]-ray[0])

                distance_intersections_tt = np.hypot(ray[0][1]-intersections_tt[:, 1],
                                                     ray[0][0]-intersections_tt[:, 0])

                return np.nanmin(distance_intersections_tt), intersections_tt[np.nanargmin(distance_intersections_tt)]
        except Exception as ex:
            # LOG.debug("ERROR -> Pose: {}".format(str(pose)))
            pass

    def get_tracing_rays(self, pose_x, pose_y, pose_angles):
        ray_start = np.array([pose_x, pose_y])
        ray_start = np.repeat(ray_start[:, np.newaxis],
                              pose_angles.shape[0], axis=1)

        unit_ray = np.array([1, 0])
        c, s = np.cos(np.radians(pose_angles)), np.sin(np.radians(pose_angles))

        R_T = np.array(((c, -s), (s, c)))
        unit_ray = unit_ray.dot(R_T)

        return np.array([ray_start, ray_start+(self.RAY_LENGTH*unit_ray)])

    def populate_views(self):
        LOG.info(" | Number of observations per grid cell: {}".format(
            self.OBS_PER_CELL))
        LOG.info(" | Precaching Views...")
        start_time = time.time()
        for cx in range(0, self.MAX_CELLS_X):
            for cy in range(0, self.MAX_CELLS_Y):
                for ca in range(0, self.MAX_CELLS_A):
                    pose = np.array(self.from_map(cx, cy, ca))

                    # Populate x, y and a values for each cell
                    self.x_values[cx, cy, ca] = pose[0]
                    self.y_values[cx, cy, ca] = pose[1]
                    self.a_values[cx, cy, ca] = pose[2]

                    # Calculate bearings and tracing rays
                    bearings = np.arange(
                        0, 360, self.RAY_TRACING_ANGLE_INCREMENT) + pose[2]

                    tracing_rays = self.get_tracing_rays(pose[0],
                                                         pose[1],
                                                         bearings)

                    # For each tracing ray, find the point of intersection and range
                    view = None
                    point = None
                    for i in range(0, self.OBS_PER_CELL):
                        try:
                            view, point = self.get_intersection(
                                tracing_rays[:, :, i], pose)
                            self.obs_views[cx, cy, ca, i] = view
                            self.obs_points_x[cx, cy, ca, i] = point[0]
                            self.obs_points_y[cx, cy, ca, i] = point[1]
                        except:
                            pass

        LOG.info(" | Precaching Time: {:.3f} secs".format(
            time.time() - start_time))

    def get_views(self, cx, cy, ca):
        return self.obs_views[cx, cy, ca]

    def print_params(self):
        LOG.info(" --------- Mapper Params ---------")

        LOG.info(" | MIN_X  : {}".format(self.MIN_X))
        LOG.info(" | MAX_X  : {} \n".format(self.MAX_X))

        LOG.info(" | MIN_Y  : {}".format(self.MIN_Y))
        LOG.info(" | MAX_Y  : {} \n".format(self.MAX_Y))

        LOG.info(" | MIN_A  : {}".format(self.MIN_A))
        LOG.info(" | MAX_A  : {} \n---".format(self.MAX_A))

        LOG.info(" | CELL_SIZE_X       : {}".format(self.CELL_SIZE_X))
        LOG.info(" | HALF_CELL_SIZE_X  : {} \n".format(self.HALF_CELL_SIZE_X))

        LOG.info(" | CELL_SIZE_Y       : {}".format(self.CELL_SIZE_Y))
        LOG.info(" | HALF_CELL_SIZE_Y  : {} \n".format(self.HALF_CELL_SIZE_Y))

        LOG.info(" | CELL_SIZE_A       : {}".format(self.CELL_SIZE_A))
        LOG.info(" | HALF_CELL_SIZE_A  : {} \n---".format(self.HALF_CELL_SIZE_A))

        LOG.info(" | MAX_CELLS_X  : {}".format(self.MAX_CELLS_X))
        LOG.info(" | MAX_CELLS_Y  : {}".format(self.MAX_CELLS_Y))
        LOG.info(" | MAX_CELLS_A  : {} \n---".format(self.MAX_CELLS_A))

        LOG.info(" | CENTER_CELL_X  : {}".format(self.CENTER_CELL_X))
        LOG.info(" | CENTER_CELL_Y  : {}".format(self.CENTER_CELL_Y))
        LOG.info(" | CENTER_CELL_A  : {} \n---".format(self.CENTER_CELL_A))

        LOG.info(" | OBS_PER_CELL                 : {}".format(self.OBS_PER_CELL))
        LOG.info(" | RAY_TRACING_ANGLE_INCREMENT  : {}".format(
            self.RAY_TRACING_ANGLE_INCREMENT))
        LOG.info(" | RAY_LENGTH                   : {} \n---".format(self.RAY_LENGTH))

        LOG.info(" --------- Mapper Params ---------\n")


class BaseLocalization():
    """A base class to perform grid localization
    """

    def __init__(self, robot, mapper):
        self.robot = robot
        self.mapper = mapper
        self.cmdr = robot.cmdr

        # Belief Matrices
        self.bel_bar = np.zeros((self.mapper.MAX_CELLS_X,
                                 self.mapper.MAX_CELLS_Y,
                                 self.mapper.MAX_CELLS_A))
        self.bel = np.zeros((self.mapper.MAX_CELLS_X,
                             self.mapper.MAX_CELLS_Y,
                             self.mapper.MAX_CELLS_A))

        # Initialize Belief Matrices
        initial_pose = (self.robot.config_params["inital_pos"][0],
                        self.robot.config_params["inital_pos"][1],
                        math.degrees(self.robot.config_params["initial_angle"]))
        self.init_grid_beliefs(*initial_pose)

        # Current data collected robot
        self.obs_range_data = None
        self.obs_bearing_data = None

        # Noise Parameters
        self.odom_trans_sigma = self.robot.config_params["localization"]["odom_trans_sigma"]
        self.odom_rot_sigma = self.robot.config_params["localization"]["odom_rot_sigma"]
        self.sensor_sigma = self.robot.config_params["localization"]["sensor_sigma"]

    # Initialize Grid Beliefs
    def init_grid_beliefs(self, x=0, y=0, a=0):
        self.init_uniform_distribution(x, y, a)

    # Initial beliefs with a uniform distribution
    def init_uniform_distribution(self, x, y, a):
        self.initial_pose = self.mapper.to_map(x, y, a)
        LOG.info("Initializing beliefs with a Uniform Distribution")

        self.bel.fill(1 / (self.bel.size))
        self.bel_bar.fill(1 / (self.bel_bar.size))

        LOG.info("Uniform Belief with each cell value: {}".format(
            self.bel[0, 0, 0]))

    # Initial belief with a point mass distribution
    def init_point_mass_distribution(self, x, y, a):
        self.initial_pose = self.mapper.to_map(x, y, a)
        LOG.info("Initial Pose: {}".format(self.initial_pose))

        LOG.info("Initializing belief with a Point mass Distribution at: {}".format(
            self.initial_pose))
        self.bel = np.zeros((self.mapper.MAX_CELLS_X,
                             self.mapper.MAX_CELLS_Y,
                             self.mapper.MAX_CELLS_A))
        self.bel[self.initial_pose] = 1

    # Gassian Function
    def gaussian(self, x, mu, sigma):
        return np.exp(-np.power(x - mu, 2) / (2*np.power(sigma, 2)))

    # Execute the rotation behavior to measure observations
    async def get_observation_data(self, rot_vel=120):
        self.obs_range_data, self.obs_bearing_data = await self.robot.perform_observation_loop(
            rot_vel)

    # Print prior belief statistics (for after prediction step) and plot data in the plotter
    def print_prediction_stats(self, plot_data=True):
        LOG.info('---------- PREDICTION STATS -----------')
        current_odom, current_gt = self.robot.get_pose()

        gt_index = self.mapper.to_map(*current_gt)
        argmax_bel_bar = get_max(self.bel_bar)
        current_prior_belief = self.mapper.from_map(*argmax_bel_bar[0])
        pos_error = np.array(current_gt) - \
            np.array(current_prior_belief)

        # Print prob as a string to prevent rounding
        LOG.info("GT index         : {}".format(gt_index))
        LOG.info("Prior Bel index  : {} with prob = {}".format(argmax_bel_bar[0],
                                                               str(argmax_bel_bar[1])[:9]))
        LOG.info(
            "POS ERROR        : ({:.3f}, {:.3f}, {:.3f})".format(*pos_error))

        # Plot data
        if(plot_data == True):
            self.cmdr.plot_gt(current_gt[0],
                              current_gt[1])
            self.cmdr.plot_odom(current_odom[0],
                                current_odom[1])
            belief_bar_marginal = np.sum(self.bel_bar, axis=2)
            self.cmdr.plot_distribution(belief_bar_marginal)
        LOG.info('---------- PREDICTION STATS -----------')
        return pos_error

    # Print belief statistics (for after update step) and plot data in the plotter
    def print_update_stats(self, plot_data=True):
        LOG.info('---------- UPDATE STATS -----------')
        current_gt = self.robot.get_pose()[1]

        gt_index = self.mapper.to_map(*current_gt)
        argmax_bel = get_max(self.bel)
        current_belief = self.mapper.from_map(*argmax_bel[0])
        pos_error = np.array(current_gt) - np.array(current_belief)

        # Print prob as a string to prevent rounding
        LOG.info("GT index      : {}".format(gt_index))
        LOG.info("Bel index     : {} with prob = {}".format(argmax_bel[0],
                                                            str(argmax_bel[1])[:9]))
        LOG.info("Bel_bar prob at index = {}".format(
            self.bel_bar[argmax_bel[0]]))

        LOG.info(
            "GT            : ({:.3f}, {:.3f}, {:.3f})".format(*current_gt))
        LOG.info("Belief        : ({:.3f}, {:.3f}, {:.3f})".format(
            *current_belief))
        LOG.info("POS ERROR     : ({:.3f}, {:.3f}, {:.3f})".format(*pos_error))

        # Plot data
        if(plot_data == True):
            self.cmdr.plot_bel(current_belief[0],
                               current_belief[1])

        LOG.info('---------- UPDATE STATS -----------')

        return pos_error

    def print_params(self):
        LOG.info(" --------- Localization Params ---------")

        LOG.info(" | initial pose    : {}".format(self.initial_pose))
        LOG.info(" | odom_trans_sigma: {}".format(self.odom_trans_sigma))
        LOG.info(" | odom_rot_sigma  : {}".format(self.odom_rot_sigma))
        LOG.info(" | sensor_sigma    : {}".format(self.sensor_sigma))

        LOG.info(" --------- Localization Params ---------\n")
