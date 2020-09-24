# Joshua Katz
# 8/27/2020
# This is a class that can take in a velocity for the robot and output
# trajectories for the legs.
#
# This will be in the form of a list of coordinates that are evenly distributed
# in space. All commands are issued to the robot at 100 Hz, so all coordinates
# are expected to have 0.01 seconds occur between them (this frequency is subject
# to change but is a good standard for now)


import numpy as np
import math

import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for 3D plots


class LegTrajectoryGenerator:
    """
    A class to compute and store body and leg trajectories

    Attributes:
    -----------
    default pose
    current_body_trajectory
    ...

    Methods
    -------
    ... list here when done

    """

    def __init__(self, default_pose):
        """
        Default constructor for the LegTrajectoryGenerator class

        Parameters:
        -----------
            default_pose : array of tuples of 2 dimensions
            current_body_trajectory : ...

        """

        self.default_pose = default_pose
        self.current_body_trajectory = None
        self.current_body_transition_trajectory = None
        self.start_pos = np.array([0, 0])
        self.x_velocity = 0
        self.y_velocity = 0
        self.angular_velocity = 0
        self.max_range = 0.10
        self.frequency = 100  # Hz

    def compute_center_of_rotation(self, vel, ang_vel):
        """


        :param vel:
        :param ang_vel:
        :return:
        """

        if ang_vel == 0:
            return None  # center does not exist for straight line motion

        if np.linalg.norm(vel) == 0:
            return self.start_pos

        # find magnitude of turning radius
        radius = np.linalg.norm(vel) / math.fabs(ang_vel)

        if ang_vel > 0:
            rot_mat = np.array([[0, -1], [1, 0]])
        else:
            rot_mat = np.array([[0, 1], [-1, 0]])

        # condense later
        # find the perpendicular velocity vector
        vel_perp = np.matmul(rot_mat, vel)
        unit_perp = vel_perp / np.linalg.norm(vel_perp)
        rad_perp = unit_perp * radius  # vector that points to the center of rotation

        # use radial perpendicular vector to compute center position of rotation

        center = self.start_pos + rad_perp
        return center

    def compute_position(self, center_pos, ang_vel, t):
        """

        :param center_pos:
        :param ang_vel:
        :param t:
        :return:
        """

        theta = ang_vel * t
        rot_mat = np.array([[math.cos(theta), -math.sin(theta)],
                            [math.sin(theta), math.cos(theta)]])
        pos = np.matmul(rot_mat, self.start_pos - center_pos) + center_pos

        return pos

    def compute_body_trajectory(self, x_vel, y_vel, ang_vel):
        """
        Computes the 2D trajectory of the body given initial velocities

        :param x_vel: float
        :param y_vel: float
        :param ang_vel: float
        :return: three arrays of floats (x coordinates, y coordinates, times)
        """

        initial_time = time.time()

        # Prepare initial conditions for body trajectory calculation
        self.x_velocity = x_vel
        self.y_velocity = y_vel
        self.angular_velocity = ang_vel

        # Create velocity vector
        vel = np.array([x_vel, y_vel])

        if np.linalg.norm(vel) == 0 and ang_vel == 0:
            print("Robot is not moving")
            exit()

        # if np.linalg.norm(vel) == 0:
        #    print("Linear velocity is 0, use different method")

        # if ang_vel == 0:
        #    print("angular velocity is 0, use different method")

        # calculate approximate maximum velocity of motion
        radius = math.sqrt(self.default_pose[0][0] * self.default_pose[0][0] +
                           self.default_pose[0][1] * self.default_pose[0][1])
        approx_max_vel = math.sqrt(
            x_vel * x_vel + y_vel * y_vel) + radius * abs(ang_vel)

        # calculate time it would take to move through range of motion
        max_time = self.max_range * 2 / approx_max_vel

        # Time from start of motion and time interval
        start_time = -max_time / 2
        final_time = max_time / 2  # seconds
        time_interval = 1 / self.frequency  # magnitude of time step

        center = self.compute_center_of_rotation(vel, ang_vel)

        # compute initial angle theta for rotation:
        # theta_init = math.atan2(rad_perp[1],rad_perp[0])

        # Compute body trajectory
        x_data = []
        y_data = []
        time_data = np.arange(start_time, final_time, time_interval)

        for t in time_data:
            if center is None:
                data_point = self.start_pos + t * vel
                x_data.append(data_point[0])
                y_data.append(data_point[1])
            else:
                data_point = self.compute_position(center, ang_vel, t)
                x_data.append(data_point[0])
                y_data.append(data_point[1])

        self.current_body_trajectory = [x_data, y_data, time_data]

        # NEW PART: TRANSITION TRAJECTORY

        # Compute transition body trajectory
        x_data = []
        y_data = []
        time_data = np.arange(0, max_time, time_interval)

        for t in time_data:
            if center is None:
                data_point = self.start_pos + t * vel
                x_data.append(data_point[0])
                y_data.append(data_point[1])
            else:
                data_point = self.compute_position(center, ang_vel, t)
                x_data.append(data_point[0])
                y_data.append(data_point[1])

        self.current_body_transition_trajectory = [x_data, y_data, time_data]

        final_time = time.time() - initial_time
        # print(final_time)

        # Plot the body trajectory
        # if center is not None:
        #     plt.scatter([center[0]], [center[1]])
        # plt.scatter([x_data], [y_data])
        # plt.axis('scaled')
        # plt.show()

        return x_data, y_data, time_data

    def convert_leg_coord_to_vector(self, coord):
        leg_pos_init = np.array(coord) + self.start_pos
        leg_vector = np.zeros(3)
        leg_vector[0:2] = leg_pos_init
        leg_vector[2] = 1
        return leg_vector

    def compute_leg_ground_trajectory(self, init_leg_coord, transition=False):
        """
        Computes ground trajectory for leg based on current body trajectory

        Given a coordinate (x,y) relative to the center of the robot, this
        function will return a list of coordinates that the foot of the leg
        should follow on the ground to achieve the current body trajectory

        :param init_leg_coord: list of 2 floats
        :return: 2 lists of floats (x data, y data)
        """

        if self.current_body_trajectory is None:
            print("Error: no body trajectory has been defined yet")
            return

        init_leg_vector = self.convert_leg_coord_to_vector(init_leg_coord)
        x_data_leg = []
        y_data_leg = []

        if transition:
            x_data = self.current_body_transition_trajectory[0]
            y_data = self.current_body_transition_trajectory[1]
            time_data = self.current_body_transition_trajectory[2]
        else:
            x_data = self.current_body_trajectory[0]
            y_data = self.current_body_trajectory[1]
            time_data = self.current_body_trajectory[2]

        # compute leg position using transformation matrix
        # For each position of the robot in time, calculate relative position of leg
        #  Calculate translation component
        # print(fr_pos_init_vector)

        for i in range(0, len(x_data)):
            t = time_data[i]
            trans = np.array([x_data[i], y_data[i]]) - self.start_pos
            # trans = np.zeros(2)
            theta_leg = self.angular_velocity * t
            rot_mat_leg = np.array(
                [[math.cos(theta_leg), -math.sin(theta_leg)],
                 [math.sin(theta_leg), math.cos(theta_leg)]])
            transformation_mat = np.zeros([3, 3])
            transformation_mat[0:2, 0:2] = rot_mat_leg
            transformation_mat[0:2, 2] = trans
            transformation_mat[2, 2] = 1
            transformation_mat_inv = np.linalg.inv(transformation_mat)

            relative_pos = np.matmul(transformation_mat_inv, init_leg_vector)

            x_data_leg.append(relative_pos[0])
            y_data_leg.append(relative_pos[1])

        return [x_data_leg, y_data_leg]

    def compute_leg_ground_trajectory_approx(self, init_leg_coord,
                                             approx_points,
                                             transition=False):

        if self.current_body_trajectory is None:
            print("Error: no body trajectory has been defined yet")
            return

        init_leg_vector = self.convert_leg_coord_to_vector(init_leg_coord)
        x_data_leg = []
        y_data_leg = []

        if transition:
            x_data = self.current_body_transition_trajectory[0]
            y_data = self.current_body_transition_trajectory[1]
            time_data = self.current_body_transition_trajectory[2]
        else:
            x_data = self.current_body_trajectory[0]
            y_data = self.current_body_trajectory[1]
            time_data = self.current_body_trajectory[2]

        # choose number of points to approximate
        # approx_points = 3  # choosing 3 for now, could be dependent on v,w

        # use total of points in body trajectory to determine indeces
        percents = np.linspace(0, 1, approx_points)
        indices = []
        for percent in percents:
            indices.append(
                int(percent * (len(self.current_body_trajectory[0]) - 1)))

        x_indices = []
        y_indices = []

        for i in indices:
            t = time_data[i]
            trans = np.array([x_data[i], y_data[i]]) - self.start_pos
            # trans = np.zeros(2)
            theta_leg = self.angular_velocity * t
            rot_mat_leg = np.array(
                [[math.cos(theta_leg), -math.sin(theta_leg)],
                 [math.sin(theta_leg), math.cos(theta_leg)]])
            transformation_mat = np.zeros([3, 3])
            transformation_mat[0:2, 0:2] = rot_mat_leg
            transformation_mat[0:2, 2] = trans
            transformation_mat[2, 2] = 1
            transformation_mat_inv = np.linalg.inv(transformation_mat)

            relative_pos = np.matmul(transformation_mat_inv, init_leg_vector)

            x_indices.append(relative_pos[0])
            y_indices.append(relative_pos[1])

        # linearly interpolate to get points in between

        x_data_leg = [x_indices[0]]
        y_data_leg = [y_indices[0]]

        for i in range(0, len(indices) - 1):
            # compute transition from i to i + 1
            n_points = indices[i + 1] - indices[i] - 1
            x_data_leg += list(
                np.linspace(x_indices[i], x_indices[i + 1], n_points + 2)[1:])
            y_data_leg += list(
                np.linspace(y_indices[i], y_indices[i + 1], n_points + 2)[1:])

        return [x_data_leg, y_data_leg]

    def compute_leg_aerial_trajectory(self, start_coord, end_coord, init_height,
                                      peak_height, n_points):
        """
        Computes aerial trajectory for a leg from start and end points and height

        Given a starting coordinate, an ending coordinate, a maximum height,
        and a number of points for the output, a list of coordinates in 3D
        space is created that lets a leg move from start to end in a triangular
        shape in the air. Assumes that start and end point are at same height

        **(Could potentially make other smoother shapes, such
        as parabolas in the future)

        :param start_coord: list of two floats (x,y)
        :param end_coord: list of two floats (x,y)
        :param height: float
        :param n_points: integer (must be greater than 0)
        :return: list of coordinates (x,y,z) in 3D space of length n_points
        """

        # General strategy: use linspace with n+2 points. x y will be perfectly
        # spaced regardless of height. z will be the same list forwards and
        # backwards, use linspace on first half of list

        x = np.linspace(start_coord[0], end_coord[0], n_points + 2)[1:-1]
        y = np.linspace(start_coord[1], end_coord[1], n_points + 2)[1:-1]

        z1 = np.linspace(init_height, peak_height,
                         math.ceil((n_points + 2) / 2))
        z2 = np.linspace(peak_height, init_height,
                         math.ceil((n_points + 2) / 2))

        if (n_points + 2) % 2 == 0:
            z = np.append(z1, z2)[1:-1]
        else:
            z = np.append(z1[:-1], z2)[1:-1]

        return x, y, z

    def compute_leg_raised_triangle_trajectory(self, start_coord, end_coord,
                                               init_height, peak_height,
                                               n_points):

        # for n points, spend n/4 of time for each vertical lift, n/2 in
        # connecting triangle

        start_3d = (start_coord[0], start_coord[1], init_height)
        end_3d = (end_coord[0], end_coord[1], init_height)

        n_vert = math.floor(n_points / 4)
        n_triangle = n_points - n_vert * 2 - 2

        vert_height = 0.5 * (peak_height - init_height) + init_height
        start_triangle = (start_coord[0], start_coord[1], vert_height)
        end_triangle = (end_coord[0], end_coord[1], vert_height)

        traj_1 = self.compute_leg_linear_trajectory(start_3d, start_triangle,
                                                    n_vert)
        traj_3 = self.compute_leg_linear_trajectory(end_triangle, end_3d,
                                                    n_vert)
        traj_2 = self.compute_leg_aerial_trajectory(
            (start_triangle[0], start_triangle[1]),
            (end_triangle[0], end_triangle[1]),
            vert_height, peak_height, n_triangle)

        lists = []
        for i in range(0, 3):
            ell = list(traj_1[i]) + [start_triangle[i]] + list(traj_2[i]) + [
                end_triangle[i]] + list(traj_3[i])
            lists.append(ell)

        print(len(lists[0]))
        print(n_points)
        # lists = [traj_1[i] + [start_triangle[i]] + traj_1[i] + [end_triangle[i]] + traj_3[i] for i in range(0,3)]

        return lists[0], lists[1], lists[2]

    def compute_leg_linear_trajectory(self, start_coord, end_coord, n_points):

        x = np.linspace(start_coord[0], end_coord[0], n_points + 2)[1:-1]
        y = np.linspace(start_coord[1], end_coord[1], n_points + 2)[1:-1]
        z = np.linspace(start_coord[2], end_coord[2], n_points + 2)[1:-1]

        return x, y, z

    def compute_leg_linear_trajectory2(self, start_coord, end_coord, n_points):
        # same as above I just want the last coordinate
        x = np.linspace(start_coord[0], end_coord[0], n_points + 2)[1:]
        y = np.linspace(start_coord[1], end_coord[1], n_points + 2)[1:]
        z = np.linspace(start_coord[2], end_coord[2], n_points + 2)[1:]

        return x, y, z

    def compute_leg_cycle_trajectory(self, init_leg_coord, init_height,
                                     max_height, ground_prop, approx=False):
        """
        Computes the ground and matching aerial trajectory for leg

        Given a coordinate (x,y) relative to the center of the robot, this
        function will return a list of coordinates that the foot of the leg
        should follow on the ground and in the air to achieve the current
        body trajectory continuously. Must also specify the initial height
        and final height as well as the proportion of time to spend on the
        ground in the full cycle

        :param init_leg_coord: list of two floats
        :param init_height: float
        :param max_height: float
        :param ground_prop: float between 0 and 1
        :param approx: Boolean
        :return: three lists of floats (x,y,z)

        """

        if approx:
            ground_traj = self.compute_leg_ground_trajectory_approx(
                init_leg_coord, 3)
        else:
            ground_traj = self.compute_leg_ground_trajectory(init_leg_coord)

        # extract starting and ending coordinates from ground trajectory
        n_ground = len(ground_traj[0])
        n_air = round((n_ground - 1) / ground_prop - n_ground + 1) - 1

        start = (ground_traj[0][0], ground_traj[1][0])
        end = (ground_traj[0][n_ground - 1], ground_traj[1][n_ground - 1])

        # CHANGE TO compute_leg_aerial trajectory FOR TRIANGLES
        aerial_traj = self.compute_leg_raised_triangle_trajectory(
            end, start, init_height, max_height, n_air)

        # aerial_traj = self.compute_leg_aerial_trajectory(
        #    end, start, init_height, max_height, n_air)

        # synthesize final trajectory cycles
        x_cycle = np.append(ground_traj[0], aerial_traj[0])
        y_cycle = np.append(ground_traj[1], aerial_traj[1])
        z_cycle = np.append(np.full(n_ground, init_height), aerial_traj[2])

        return x_cycle, y_cycle, z_cycle

    def compute_march_cycle(self, init_leg_coord, init_height,
                            max_height, ground_prop, frequency):

        # each leg starts on the ground for ground_prop time, moves straight up,
        # and then moves straight down

        total_time = 1 / frequency
        velocity = 2 * (max_height - init_height) / (
                    (1 - ground_prop) * total_time)
        distance = velocity / self.frequency
        n_air = int(2 * (max_height - init_height) / distance)
        n_total = int(n_air / (1 - ground_prop))
        n_ground = n_total - n_air

        aerial = self.compute_leg_aerial_trajectory(init_leg_coord,
                                                    init_leg_coord, init_height,
                                                    max_height, n_air)

        x = list(np.linspace(init_leg_coord[0], init_leg_coord[0], n_total))
        y = list(np.linspace(init_leg_coord[1], init_leg_coord[1], n_total))
        z_ground = np.linspace(init_height, init_height, n_ground)
        z = list(z_ground) + list(aerial[2])

        return x, y, z



if __name__ == "__main__":

    # Set initial leg position(s) (default pose)
    # Order is: FL, FR, BL, BR
    leg_positions = [(-0.135, 0.15), (0.135, 0.15), (-0.135, -0.15),
                     (0.135, -0.15)]

    generator = LegTrajectoryGenerator(leg_positions)

    # initial velocity conditions
    x_velocity = 0  # sideways velocity
    y_velocity = 1  # forward velocity
    ang_velocity = 1  # angular velocity

    generator.compute_body_trajectory(x_velocity, y_velocity, ang_velocity)

    generator.compute_body_trajectory(0, 0.2, 1)

    # Compute and store example leg trajectories
    start_time = time.time()
    leg_data = []
    for leg in leg_positions:
        leg_data.append(generator.compute_leg_ground_trajectory(leg))
    print(time.time() - start_time)

    # Make Plots
    plt.scatter([0], [0])
    for leg in leg_data:
        plt.scatter(leg[0], leg[1])
    plt.axis('scaled')
    plt.show()

    # aerialx, aerialy, aerialz = generator.compute_leg_aerial_trajectory([0, 0], [1, 1], 0, 0.02, 40)
    aerialx, aerialy, aerialz = generator.compute_march_cycle(
        leg_positions[0], 0, 0.05, 0.5, 1)
    #aerialx, aerialy, aerialz = generator.compute_leg_cycle_trajectory(
    #   leg_positions[0], 0, 0.05, 0.8, True)

    print("x: ", aerialx)
    print("y: ", aerialy)
    print("z: ", aerialz)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(aerialx, aerialy, aerialz, c='r', marker='o')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

    data = list(zip(aerialx, aerialy, aerialz))
    print(data)
