# Joshua Katz
# 8/30/20

import time
import os
import numpy as np
from LegTrajectoryGenerator import LegTrajectoryGenerator
from Simulator import Simulator
import keyboard


class TrajectoryExecutor:

    def __init__(self):

        self.modes = ["idle", "idle", "idle", "idle"]
        self.cycles = []
        self.transitions = []
        self.trajectories = []
        self.clock_index = 0
        self.clock_max = 0
        self.transition_index = 0
        self.phases = [0, 0.5, 0.75, 0.25]
        self.ground_prop = 0.75
        self.default_pose = [(-0.135, 0.15), (0.135, 0.15), (-0.135, -0.15),
                             (0.135, -0.15)]
        
        self.low = -0.2
        self.high = -0.15

        self.current_position = None
        self.stand_position = [(-0.135, 0.15, -0.2), (0.135, 0.15, -0.2), (-0.135, -0.15, -0.2), (0.135, -0.15, -0.2)]
        self.leg_trajectory_generator = LegTrajectoryGenerator(
            self.default_pose)
        self.current_velocity = (0, 0, 0)

    def choose_gait(self, x_vel, y_vel, ang_vel):

        # find out if x, y, or angular velocity is "dominant"
        # linear_mag = np.linalg.norm(np.array(x_vel, y_vel))

        # if linear_mag
        # gp is percent time on ground
        # phases preportion at which the leg starts in a cycle
        # [FL Fr Bl BR]


        rotational = True
        if ang_vel == 0 or max(x_vel, y_vel) / ang_vel > 0.1:
            rotational = False

        if not rotational:
            self.ground_prop = 0.9  # subject to change
            if abs(y_vel) > abs(x_vel):
                if y_vel > 0:
                    if y_vel > 0.3:
                        self.phases = [0, 0.5, 0.5, 0]
                    else:
                        self.phases = [0, 0.5, 0.25, 0.75]
                else:
                    self.phases = [0, 0.5, 0.75, 0.25]
            else:
                if x_vel > 0:
                    self.phases = [0, 0.75, 0.5, 0.25]
                else:
                    self.phases = [0, 0.25, 0.5, 0.75]

        else:
            self.ground_prop = 0.90
            if ang_vel > 0:
                self.phases = [0, 0.25, 0.75, 0.5]
            else:
                self.phases = [0, 0.75, 0.25, 0.5]

        # if round(y_vel, 1) >= 0.7:
        #    # print("gallop")
        #    self.phases = [0, 0, 0.5, 0.5]
        #   self.ground_prop = 0.55

        # elif y_vel >= 0.4:
        #    # print("trot")
        #    self.phases = [0, 0.5, 0.5, 0]
        #   self.ground_prop = 0.6

        # else:
        #    # print("walk")
        #    self.phases = [0, 0.5, 0.75, 0.25]
        #   self.ground_prop = 0.75

        return self.phases, self.ground_prop

    def change_movement_speed(self, x_vel, y_vel, ang_vel):

         #get current robot position if possible
        #if self.current_position is not None:
        #    leg_positions = self.current_position
        #   # print("changed to current position")
        #else:
        #    self.current_position = leg_positions

        if self.current_velocity[0] == x_vel \
                and self.current_velocity[1] == y_vel \
                and self.current_velocity[2] == ang_vel:
            # print("velocity has not changed")
            return

        # if speed is 0, stop moving the legs
        if x_vel == 0 and y_vel == 0 and ang_vel == 0:

            # set modes based on findings
            self.modes = []  # set all legs to transition

            # check if legs are in air
            for i in range(0, len(self.current_position)):
                height = round(self.current_position[i][2], 4)
                if height > self.low:
                    self.modes.append("stabilize_transition")
                    self.transitions[i] = []
                else:
                    self.modes.append("idle")

            # self.modes = ["idle", "idle", "idle", "idle"]
            self.current_velocity = (x_vel, y_vel, ang_vel)
            self.transition_index = 0
            return

        # print(leg_positions)

        # decide heights of aerial trajectories
        #self.low = -0.2
        #self.high = -0.15

        # decide gait phases and ground proportion, stored in object
        self.choose_gait(x_vel, y_vel, ang_vel)

        # compute body trajectories

        self.leg_trajectory_generator.compute_body_trajectory(x_vel, y_vel,
                                                              ang_vel)
        start_time = time.time()
        # compute leg cycle trajectories
        self.cycles = []
        for leg in self.default_pose:
            traj = self.leg_trajectory_generator.compute_leg_cycle_trajectory(
                leg, self.low, self.high, self.ground_prop, True)
            self.cycles.append(list(zip(traj[0], traj[1], traj[2])))

        print("")
        print("Cycle trajectory time: " + str(time.time() - start_time))
        start_time = time.time()

        # FOR NOW, PRINT COORDINATES IN LEG CYCLE AND END

        # print(self.cycles[0])
        # print(self.cycles[1][65])
        # print(self.cycles[2][96])
        # print(self.cycles[3][34])

        # compare leg positions to cycle to determine best cycle starting point

        # for each leg cycle, find the coordinate that is closest to the
        # starting position
        min_dists = []
        min_idxs = []

        for i in range(0, len(self.current_position)):
            min_dist = self.distance_between_coords(self.cycles[i][0],
                                                    self.current_position[i])
            min_idx = 0
            for j in range(0, len(self.cycles[
                                      i])):  # every other approx to improve performance?
                dist = self.distance_between_coords(self.cycles[i][j],
                                                    self.current_position[i])
                if dist < min_dist:
                    min_dist = dist
                    min_idx = j
            min_dists.append(min_dist)
            min_idxs.append(min_idx)

        # print("Ideal leg indexes", min_idxs)
        # print(min_dists)
        print(
            "Distance index calculation time: " + str(time.time() - start_time))
        start_time = time.time()

        # Using the phases and ground proportion, find the clock index that
        # makes the indexes all match as closely as possible

        # compute phase shift for each leg

        phase_shifts = []
        for phase in self.phases:
            phase_shifts.append(int(phase * len(self.cycles[0])))

        # iterate through all possible clock indexes and select the one that
        # makes the total difference of indexes the lowest:

        idxs = phase_shifts.copy()  # at t = 0, the indexes are the phase shifts
        idx_fit = idxs.copy()
        min_time = 0
        min_diff = list(x - y for x, y in zip(idxs, min_idxs))
        min_mag = np.linalg.norm(np.array(min_diff))

        for t in range(0, len(self.cycles[0])):
            difference = list(x - y for x, y in zip(idxs, min_idxs))
            diff_mag = np.linalg.norm(np.array(difference))
            # print(diff_mag)
            if diff_mag < min_mag:
                min_time = t
                min_diff = difference  # might not need this one
                min_mag = diff_mag
                idx_fit = idxs.copy()
            # update indexes for next iteration by adding one and modding
            for i in range(0, len(idxs)):
                idxs[i] = (idxs[i] + 1) % len(self.cycles[0])

        print("mag: " + str(min_mag))

        # new idea. If the command is very similar to the previous command,
        # just keep the last clock index and continue in cycle mode...

        # dot product of unit vectors!

        new_vel = np.array([x_vel, y_vel, ang_vel])
        new_vel = new_vel / np.linalg.norm(new_vel)

        old_vel = np.array(self.current_velocity)

        if np.linalg.norm(old_vel) != 0:
            old_vel = old_vel / np.linalg.norm(old_vel)

        dot = np.dot(new_vel, old_vel)

        # command is similar, keep relative cycle position
        if dot >= 0.95:
            self.clock_index = round(
                self.clock_index / self.clock_max * (len(self.cycles[0]) - 1))

        # command is not similar, use optimization
        else:
            self.clock_index = min_time

        self.clock_max = len(self.cycles[0]) - 1

        print("Cycle Optimization time: " + str(time.time() - start_time))
        start_time = time.time()

        # new cycles are sufficiently close to original cycles
        if min_mag < 3:
            self.modes = ["cycle", "cycle", "cycle", "cycle"]
            print("Continuing in cycle mode")
            self.current_velocity = (x_vel, y_vel, ang_vel)
            return

        # print("Best index fit: " + str(idx_fit))
        # print("Corresponding clock index: " + str(self.clock_index))

        # optional: compute body transition trajectory and leg trajectories

        # Determine what portion of transition ground trajectories are in range
        # NOTE: this might need to happen on a different cycle, computation
        # is starting to become a bit intense for one cycle of commands...

        # for each point in transition trajectory, check distance to
        # corresponding default pose leg position

        # compute leg transition trajectories
        self.transitions = []
        for leg in self.current_position:
            traj = self.leg_trajectory_generator.compute_leg_ground_trajectory_approx(
                [
                    leg[0], leg[1]], 3, True)
            z = list(np.full(len(traj[0]), self.low))
            self.transitions.append(list(
                zip(traj[0], traj[1], z)))  # get rid of first point in list

        print("Transition calculation time: " + str(time.time() - start_time))

        '''max_idxs = []
        for i in range(0, len(self.transitions)):
            if round(self.transitions[i][0][2],
                     4) > self.low:  # leg is in air, range is irr
                max_idxs.append(0)  # append 0, no range available, aerial
                continue
            for j in range(0, len(self.transitions[i])):

                point = [self.transitions[i][j][0], self.transitions[i][j][1]]
                dist = self.distance_between_coords(self.default_pose[i], point)
                if dist > self.leg_trajectory_generator.max_range:
                    max_idxs.append(j)
                    break
            if len(max_idxs) < i + 1:
                max_idxs.append(len(self.transitions[i]) - 1)'''

        # print("maximum steps for each transition before out of range: "
        #     + str(max_idxs))

        # lop off parts of transition that are unusable (could also just ignore)
        # for i in range(0, len(self.transitions)):
        #    self.transitions[i] = self.transitions[i][: max_idxs[i]]
        #    print(len(self.transitions[i]))

        self.transition_index = 0

        # set modes based on findings
        self.modes = []  # set all legs to transition

        for i in range(0, len(self.transitions)):
            height = round(self.current_position[i][2], 4)
            if height > self.low:
                self.modes.append("aerial_transition")
                self.transitions[i] = []
            else:
                self.modes.append("ground_transition")

        self.current_velocity = (x_vel, y_vel, ang_vel)
        print(self.modes)
        print(self.clock_index)

        # calculate aerial trajectories? or maybe do this elsewhere?
        # print(self.modes)

        # test to eliminate transition
        # self.modes = ["cycle", "cycle", "cycle", "cycle"]
        # self.clock_index = 0

    def aerial_to_cycle_trajectory(self, phase_idxs, i):
        # compute aerial trajectory
        # for now, I will use a straight line!

        # there are two possibilities
        # 1. cycle is also currently in air, go to end of cycle point
        # * if there isn't enough time, go to ground anyway
        # 2. cycle is on ground, intercept as quickly as possible

        # check if corresponding cycle is in air
        # AND if there is enough time

        n_points = len(self.cycles[i]) - phase_idxs[i]
        distance = self.distance_between_coords(self.current_position[i],
                                                self.cycles[0])
        speed = distance / (n_points * 0.01)  # speed to get to goal
        if round(self.cycles[i][phase_idxs[i]][2]) > self.low and speed < 0.5:
            # n_points > round(0.1 * len(self.cycles[i])):

            # set interception point at beginning of cycle
            target = self.cycles[i][0]

        # if there isn't enough time, meet on ground or in mid air instead
        else:
            # time gap is 0.1, so points is -2
            n_points = round(0.1 * len(self.cycles[i]))  # not so bad
            target_index = (phase_idxs[i] + n_points + 1) % len(self.cycles[i])
            target = self.cycles[i][target_index]

        aerial = self.leg_trajectory_generator.compute_leg_linear_trajectory(
            self.current_position[i], target, n_points)

        # self.transitions[i] = list(zip(aerial[0], aerial[1], aerial[2]))
        aerial_coords = list(zip(aerial[0], aerial[1], aerial[2]))
        return aerial_coords

    def max_range_trajectory(self, phase_idxs, i):
        print(str(i) + " is out of range")

        # choose a time in the future to intersect
        n_points = round(0.1 * len(self.cycles[i]))
        target_index = (phase_idxs[i] + n_points + 1) % len(self.cycles[i])
        target = self.cycles[i][target_index]

        if round(target[2], 4) > self.low:
            aerial = self.leg_trajectory_generator.compute_leg_linear_trajectory(
                self.current_position[i], target, n_points)
        else:
            start_coord = (
                self.current_position[i][0], self.current_position[i][1])
            end_coord = (target[0], target[1])
            aerial = self.leg_trajectory_generator.compute_leg_aerial_trajectory(
                start_coord, end_coord, self.low, self.high, n_points)

        aerial_coords = list(zip(aerial[0], aerial[1], aerial[2]))
        return aerial_coords

    def get_next_command(self):
        """
        returns the next set of command coordinates based on leg modes
        """
        commands = []

        # use clock to compute cycle indexes (this can be more efficient, +1 etc)
        phase_idxs = []
        for phase in self.phases:
            phase_idxs.append(
                int((self.clock_index + phase * len(self.cycles[0])) % len(
                    self.cycles[0])))

        for i in range(0, len(self.modes)):

            if self.modes[i] == "idle":
                commands.append(self.current_position[i])

            elif self.modes[i] == "stabilize_transition":
                if len(self.transitions[i]) == 0:
                    start_coord = self.current_position[i]
                    end_coord = [self.current_position[i][0],
                                 self.current_position[i][1], self.low]
                    n_points = round(0.1 * len(self.cycles[i]))
                    aerial = self.leg_trajectory_generator. \
                        compute_leg_linear_trajectory(start_coord, end_coord,
                                                      n_points)
                    self.transitions[i] = list(
                        zip(aerial[0], aerial[1], aerial[2]))
                    self.transitions[i].append(end_coord)  # include final point
                    # print(self.transitions[i])

                # pass on transition trajectory as normal
                commands.append(self.transitions[i][self.transition_index])

                # end of stabilizing transition complete, transfer to idle
                if self.transition_index == len(self.transitions[i]) - 1:
                    self.modes[i] = "idle"


            # if leg is in cycle mode, just return its cycle coordinate
            elif self.modes[i] == "cycle":
                commands.append(self.cycles[i][phase_idxs[i]])

            # if leg is in ground transition mode:
            elif self.modes[i] == "ground_transition":

                # if cycle is beginning to aerial, switch to aerial
                # height = round(self.cycles[i][phase_idxs[i]][2], 4)

                # current position 2D used later ...
                try:
                    coord = (
                        self.current_position[i][0],
                        self.current_position[i][1])
                except:
                    coord = 0
                if round(self.cycles[i][phase_idxs[i]][2], 4) > self.low:
                    self.modes[i] = "aerial_transition"
                    # print("switching modes!")

                    # compute aerial transition
                    start = self.current_position[i]
                    end = self.cycles[i][0]
                    n_points = len(self.cycles[i]) - phase_idxs[
                        i]  # check later

                    # if there is not time to make the transition
                    if n_points <= 0.1 * len(self.cycles[i]):
                        # increase the number of points and reach
                        # different part of cycle
                        n_points = round(0.1 * len(self.cycles[i]))
                        target_index = (phase_idxs[i] + n_points + 1) % len(
                            self.cycles[i])
                        end = self.cycles[i][target_index]

                    aerial_transition = self.leg_trajectory_generator.compute_leg_aerial_trajectory(
                        start, end, self.low, self.high, n_points)

                    aerial_coords = list(
                        zip(aerial_transition[0], aerial_transition[1],
                            aerial_transition[2]))

                    # cut off unneeded part of ground trans and add aerial
                    # this makes the transition clocks line up
                    self.transitions[i] = self.transitions[i][
                                          :self.transition_index] + aerial_coords

                    commands.append(self.transitions[i][self.transition_index])

                # temporary fix: just go to next ground point
                # could possibly detect range errors here

                # If the leg goes out of range...
                elif self.distance_between_coords(coord, self.default_pose[
                    i]) > self.leg_trajectory_generator.max_range:
                    self.modes[i] = "aerial_transition"
                    aerial_coords = self.max_range_trajectory(phase_idxs, i)
                    self.transitions[i] = self.transitions[i][
                                          :self.transition_index] + aerial_coords

                    commands.append(self.transitions[i][self.transition_index])
                else:
                    commands.append(self.transitions[i][self.transition_index])

            elif self.modes[i] == "aerial_transition":
                if len(self.transitions[i]) == 0:
                    self.transitions[i] = self.aerial_to_cycle_trajectory(
                        phase_idxs, i)

                # pass on transition trajectory as normal
                commands.append(self.transitions[i][self.transition_index])

                # end of aerial transition complete, transfer to cycle
                if self.transition_index == len(self.transitions[i]) - 1:
                    self.modes[i] = "cycle"

            else:
                print("Mode not recognized")
                exit()

        # update clocks
        # if self.modes[0] != "idle":
        self.transition_index += 1
        self.clock_index = (self.clock_index + 1) % (self.clock_max + 1)

        # self.recent_command
        distance = self.distance_between_coords(commands[0],
                                                self.current_position[0])
        # print(distance)
        if distance > 0.02:
            print("error: discontinuity")
            print(commands[0])
            print(self.current_position[0])
            print(self.modes)

            # time.sleep(10)

        self.current_position = commands
        # print(commands[0])
        # print(self.modes[0])

        return commands





    @staticmethod
    def distance_between_coords(a, b):
        difference = np.array(list(x - y for x, y in zip(a, b)))
        mag = np.linalg.norm(difference)
        return mag


if __name__ == "__main__":
    # Create and initiate simulator
    current_dir = os.getcwd()
    sep = os.path.sep
    my_urdf = current_dir + sep + "Phantom" + sep + "urdf" + sep + "Phantom_connor_edits.urdf"

    sim = Simulator(True)
    sim.load_kinematics_urdf(my_urdf)
    sim.load_gui_urdf(my_urdf)

    # get initial position from simulator
    # init_pos = []
    # fk = sim.compute_multi_fk([3, 7, 11, 15])
    # for leg in fk:
    #     print(leg[0])
    #     init_pos.append(leg[0])



    exec = TrajectoryExecutor()
    #exec.current_position = init_pos

    exec.current_position = exec.stand_position
    exec.change_movement_speed(0, 0.1, 0) #makes cycles exist
    exec.change_movement_speed(0, 0, 0)
    # exec.change_movement_speed(0, 0.2, 0)

    #exec.change_movement_speed(0.2, 0.0, 0)

    # print(len(exec.cycles[0]))
    # for x in range(0,150):
    #    print(x, exec.modes[0])
    #    print(exec.get_next_command()[0])
    #    print(x, exec.modes[0])
    #    print("")

    # main loop

    default_3d_legs = [(-0.135, 0.15, -0.2), (0.135, 0.15, -0.2),
                       (-0.135, -0.15, -0.2), (0.135, -0.15, -0.2)]

    # Let the robot fall and settle
    # for x in range(0,1000):
    #   sim.step_gui_sim()
    #    time.sleep(1/240)

    for x in range(0, 10000000):
        start_time = time.time()

        # if x == 500:
        #    exec.change_movement_speed(0, -0.2, 0)
        # if x == 1000:
        #    exec.change_movement_speed(0, 0.2, 0)
        # if x == 1500:
        #    exec.change_movement_speed(0, -0.2, 0)
        # if x == 2000:
        #   exec.change_movement_speed(0, 0.2, 0)

        y = 0
        X = 0
        a = 0

        if x % 10 == 0:     #   <---   JOSH why is this not lower case x ?      ------------------------------------------
            if keyboard.is_pressed("up"):
                # exec.change_movement_speed(0, 0.3, 0, default_3d_legs)
                y = 0.3
            if keyboard.is_pressed("left"):
                # exec.change_movement_speed(-0.2, 0, 0, default_3d_legs)
                X = -0.3
            if keyboard.is_pressed("down"):
                # exec.change_movement_speed(0, -0.2, 0, default_3d_legs)
                y = -0.3
            if keyboard.is_pressed("right"):
                # exec.change_movement_speed(0.2, 0, 0, default_3d_legs)
                X = 0.3
            if keyboard.is_pressed("q"):
                # exec.change_movement_speed(0, 0, 1, default_3d_legs)
                a = 1
            if keyboard.is_pressed("e"):
                # exec.change_movement_speed(0, 0, -1, default_3d_legs)
                a = -1

            exec.change_movement_speed(X, y, a)

        # if x > 2000 and x % 10 == 0:
        #    exec.change_movement_speed(0, 0.2, 0, default_3d_legs)
        #    pass

        command = exec.get_next_command()

        # compute ik
        ik = sim.compute_multi_ik([3, 7, 11, 15], command)

        # command robot
        moving_joints = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
        sim.set_robot_pos([0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14], ik)

        #sim.center_camera()

        sim.step_gui_sim()


        end_time = time.time() - start_time
        if end_time < 0.01:
            time.sleep(1 / 240)
        else:

            print("CLOCK SKIPPED!")
            print(end_time)

