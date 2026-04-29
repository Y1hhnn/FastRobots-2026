import math
import numpy as np
from localization import BaseLocalization
import time
from copy import deepcopy
from utils import get_max, setup_logging


LOG = setup_logging("localization_extras.log")


class Localization(BaseLocalization):
    def __init__(self, robot, mapper):
        super().__init__(robot, mapper)

    def compute_control(self, cur_pose, prev_pose):
        """ Given the current and previous odometry poses, this function extracts
        the control information based on the odometry motion model.

        Args:
            cur_pose  ([Pose]): Current Pose
            prev_pose ([Pose]): Previous Pose

        Returns:
            [delta_rot_1]: Rotation 1  (degrees)
            [delta_trans]: Translation (meters)
            [delta_rot_2]: Rotation 2  (degrees)
        """
        delta_rot_1 = self.mapper.normalize_angle(math.degrees(np.arctan2(cur_pose[1]-prev_pose[1],
                                                                          cur_pose[0]-prev_pose[0])) - prev_pose[2])
        delta_trans = math.hypot(cur_pose[1]-prev_pose[1],
                                 cur_pose[0]-prev_pose[0])
        delta_rot_2 = self.mapper.normalize_angle(
            cur_pose[2] - prev_pose[2] - delta_rot_1)

        return delta_rot_1, delta_trans, delta_rot_2

    # In world coordinates
    def odom_motion_model(self, cur_pose, prev_pose, u):
        """ Odometry Motion Model

        Args:
            cur_pose  ([Pose]): Current Pose
            prev_pose ([Pose]): Previous Pose
            (rot1, trans, rot2) (float, float, float): A tuple with control data in the format
                                                    format (rot1, trans, rot2) with units (degrees, meters, degrees)


        Returns:
            prob [float]: Probability p(x'|x, u)
        """
        exp_delta_rot_1, exp_delta_trans, exp_delta_rot_2 = self.compute_control(cur_pose,
                                                                                 prev_pose)

        prob = (self.gaussian(self.mapper.normalize_angle(exp_delta_rot_1-u[0]), 0, self.odom_rot_sigma)
                * self.gaussian(exp_delta_trans, u[1], self.odom_trans_sigma)
                * self.gaussian(self.mapper.normalize_angle(exp_delta_rot_2-u[2]), 0, self.odom_rot_sigma))

        return prob

    def prediction_step(self, cur_odom, prev_odom):
        """ Prediction step of the Bayes Filter.
        Update the probabilities in self.bel_bar based on self.bel from the previous time step and the odometry motion model.

        Args:
            cur_odom  ([Pose]): Current Pose
            prev_odom ([Pose]): Previous Pose
        """
        LOG.info("Prediction Step")
        start_time = time.time()

        # Angles in prev_odom and odom are in degrees
        # Angles in u are in degrees
        u = self.compute_control(cur_odom, prev_odom)

        self.bel_bar = np.zeros((self.mapper.MAX_CELLS_X,
                                 self.mapper.MAX_CELLS_Y,
                                 self.mapper.MAX_CELLS_A))

        for prev_x in range(0, self.mapper.MAX_CELLS_X):
            for prev_y in range(0, self.mapper.MAX_CELLS_Y):
                for prev_a in range(0, self.mapper.MAX_CELLS_A):
                    if(self.bel[prev_x, prev_y, prev_a] > 0.0001):
                        for cur_x in range(0, self.mapper.MAX_CELLS_X):
                            for cur_y in range(0, self.mapper.MAX_CELLS_Y):
                                for cur_a in range(0, self.mapper.MAX_CELLS_A):
                                    self.bel_bar[cur_x, cur_y, cur_a] = self.bel_bar[cur_x, cur_y, cur_a] + (self.odom_motion_model(self.mapper.from_map(cur_x,
                                                                                                                                                         cur_y,
                                                                                                                                                         cur_a),
                                                                                                                                    self.mapper.from_map(prev_x,
                                                                                                                                                         prev_y,
                                                                                                                                                         prev_a),
                                                                                                                                    u) * self.bel[prev_x, prev_y, prev_a])

        self.bel_bar = self.bel_bar / np.sum(self.bel_bar)
        LOG.info(" | Prediction Time: {:.3f} secs".format(
            time.time() - start_time))

    def update_step(self):
        """ Update step of the Bayes Filter.
        Update the probabilities in self.bel based on self.bel_bar and the sensor model.
        """
        LOG.info("Update Step")
        start_time = time.time()

        self.bel = deepcopy(self.bel_bar)

        for i in range(0, self.mapper.OBS_PER_CELL):
            self.bel = self.bel * self.gaussian(self.mapper.obs_views[:, :, :, i] - self.obs_range_data[i][0],
                                                0,
                                                self.sensor_sigma)
            self.bel = self.bel / np.sum(self.bel)

        LOG.info("     | Update Time: {:.3f} secs".format(
            time.time() - start_time))

    # Plot the odometry and prior belief distribution in the plotter
    def plot_prediction_step_data(self):
        # Uncomment the following lines to get the robot odometry and plot it in the plotter
        # current_odom = self.robot.get_pose()

        # Plot data
        # self.cmdr.plot_odom(current_odom[0],
        #                     current_odom[1])
        belief_bar_marginal = np.sum(self.bel_bar, axis=2)
        self.cmdr.plot_distribution(belief_bar_marginal)

    # Plot belief in the plotter
    def plot_update_step_data(self, plot_data=True):
        argmax_bel = get_max(self.bel)
        current_belief = self.mapper.from_map(*argmax_bel[0])

        # Print prob as a string to prevent rounding
        LOG.info("Bel index     : {} with prob = {}".format(argmax_bel[0],
                                                            str(argmax_bel[1])[:9]))
        LOG.info("Bel_bar prob at index = {}".format(
            self.bel_bar[argmax_bel[0]]))
        LOG.info("Belief        : ({:.3f}, {:.3f}, {:.3f})".format(
            *current_belief))

        # Plot data
        self.cmdr.plot_bel(current_belief[0],
                           current_belief[1])
