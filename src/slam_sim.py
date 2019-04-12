#!/usr/bin/env python
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
import sys
from matplotlib import colors
from matplotlib import animation
from am_driver.msg import Loop
from am_driver.msg import WheelEncoder
from gazebo_msgs.msg import ModelStates


class BoundarySensor(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.accum = 0
        self.value = 0
        self.relative = 0


class SlamSim(object):

    def __init__(self):

        self.start_time = None
        # Set to negative to disable node time out
        self.time_out = -1

        self.update_rate = 5
        self.interval_time = 0
        self.observation_count = 0

        self.plot_interval = 2
        self.plot_timer = 0

        # FC, FR, RL, RR
        self.front_center = BoundarySensor(0.3, 0)
        self.front_right = BoundarySensor(0.3, -0.07)
        self.rear_left = BoundarySensor(-0.15, 0.12)
        self.rear_right = BoundarySensor(-0.15, -0.12)

        self.occupancy_grid = np.array([[0.0]])
        self.occupancy_grid_global = np.array([[0.0]])

        #in meters, size of boundary area plotted
        # self.boundary_size = 0.25

        self.grid_resolution = 3
        # width/height of grid in meters
        self.grid_size = 15
        self.grid_size_global = 50
        self.cell_count = self.grid_resolution * self.grid_size
        self.cell_count_global = self.grid_resolution * self.grid_size_global
        self.origin_x = self.origin_y = round(self.cell_count/2)
        self.origin_x_global = self.origin_y_global = round(self.cell_count_global/2)

        self.occupancy_grid = np.lib.pad(self.occupancy_grid, ((0, self.cell_count - 1), (0, self.cell_count - 1)), "constant", constant_values=(0.0))
        self.occupancy_grid_global = np.lib.pad(self.occupancy_grid_global, ((0, self.cell_count_global - 1), (0, self.cell_count_global - 1)), "constant", constant_values=(0))

        # State variables
        self.state = 0

        self.start_x = 0
        self.start_y = 0
        self.start_ori = 0

        self.x = 0
        self.y = 0
        self.ori = 0

        self.prev_tick = None

        self.wheel_l_accum = 0
        self.wheel_r_accum = 0
        self.time = 0

        self.k_poses = 3
        self.poses_maps = np.array([[[0.0]]])
        self.poses_maps = np.lib.pad(self.poses_maps, ((0, pow(self.k_poses,3) - 1), (0, self.cell_count - 1), (0, self.cell_count - 1)), "constant", constant_values=(0.0))
        self.poses = np.array([[0.0]])
        self.poses = np.lib.pad(self.poses, ((0, pow(self.k_poses, 3) - 1), (0, 4 - 1)), "constant", constant_values=(0.0))
        self.predicted_pose = np.array([0.0, 0.0, 0.0])
        self.best_pose = np.array([0.0, 0.0, 0.0])
        self.best_map = np.array([[0.0]])
        self.best_map = np.lib.pad(self.best_map, ((0, self.cell_count - 1), (0, self.cell_count - 1)), "constant", constant_values=(0.0))

        self.n_observations = 3*self.update_rate
        self.local_map_data = np.zeros(shape=(self.n_observations, 5))
        self.local_map_index = 0
        self.n_current = 0

        # make a color map of fixed colors
        self.cmap = colors.ListedColormap(['white', 'green', 'red', 'blue'])
        self.green = 1
        self.white = 0
        self.red = -1
        self.blue = 1
        self.bounds = [-1,-0.75,-0.5,-0.25,0,0.25,0.5,0.75,1]
        self.certainty_max = 0.95
        self.norm = colors.BoundaryNorm(self.bounds, self.cmap.N)

    def update(self):

        if self.state == 0:
            # Reset local map, and observations, go to state 1
            self.occupancy_grid.fill(0)
            self.n_current = 0
            self.state = 1

            self.start_x = self.best_pose[0]
            self.start_y = self.best_pose[1]
            self.start_ori = self.best_pose[2]

            # reset values for next iteration
            self.local_map_data = np.zeros(shape=(self.n_observations, 5))
            self.local_map_index = 0
            self.wheel_l_accum = 0
            self.wheel_r_accum = 0

        elif self.state == 1:
            self.n_current += 1

            (self.x, self.y, self.ori) = odometry(self.wheel_l_accum, self.wheel_r_accum, self.x, self.y, self.ori)

            self.wheel_l_accum = 0
            self.wheel_r_accum = 0

            if self.n_current <= self.n_observations:
                self.accum_data(self.x, self.y, self.ori)
            else:
                self.state = 2
        elif self.state == 2:
            self.pose_generation()
            self.state = 3
        elif self.state == 3:
            self.match_maps()
            self.update_global()
            self.state = 0

    # Update global map with data from the best local map chosen
    def update_global(self):

        x_offset = math.floor(self.start_x*self.grid_resolution) + math.floor(( len(self.occupancy_grid_global)/2 ) - (len(self.occupancy_grid)/2 ))
        y_offset = math.floor(self.start_y*self.grid_resolution) + math.floor(( len(self.occupancy_grid_global)/2 ) - ( len(self.occupancy_grid)/2 ))

        print("x offset: "+str(x_offset)+", y offset: "+str(y_offset))

        for i in range(len(self.best_map)):
            for j in range(len(self.best_map[0])):
                point_local = self.best_map[j, i]
                point_global = self.occupancy_grid_global[int(j+y_offset), int(i+x_offset)]
                if not self.occupancy_grid_global[int(j+y_offset), int(i+x_offset)] == self.blue and not point_local == 0 and not (0 <= point_local < (self.green)):
                    self.occupancy_grid_global[int(j+y_offset), int(i+x_offset)] = point_local
                elif 0 < point_local < self.green and 0 <= point_global < self.green:
                    if point_local > point_global:
                        self.occupancy_grid_global[int(j + y_offset), int(i + x_offset)] = point_local
                elif self.red <= point_local < 0 and self.red < point_global <0:
                    if point_local < point_global:
                        self.occupancy_grid_global[int(j + y_offset), int(i + x_offset)] = point_local
                elif (point_local < 0 and point_global > 0) or (point_local > 0 and point_global < 0):
                    self.occupancy_grid_global[int(j + y_offset), int(i + x_offset)] = (point_global + point_local)/2.0


    # compare all generated local maps to global and select best
    def match_maps(self):

        x_offset = math.floor(self.start_x * self.grid_resolution) + math.floor((len(self.occupancy_grid_global) / 2) - (len(self.poses_maps[0]) / 2))
        y_offset = math.floor(self.start_y * self.grid_resolution) + math.floor((len(self.occupancy_grid_global) / 2) - (len(self.poses_maps[0]) / 2))

        best_index = int(len(self.poses_maps)/2)
        best_rmse = sys.float_info.max
        mid_rmse = sys.float_info.max
        # print("i"+str(len(self.poses_maps))+", j"+str(len(self.poses_maps[0]))+", k"+str(len(self.poses_maps[1])))



        # Calculate RMSE for each map, and find best
        for i in range(len(self.poses_maps)):
            sum_count = 0
            sum = 0
            for j in range(len(self.poses_maps[0])):
                for k in range(len(self.poses_maps[0])):
                    global_point = self.occupancy_grid_global[int(k + y_offset), int(j + x_offset)]
                    local_point = self.poses_maps[i,k,j]
                    if not global_point == 0 and self.red <= local_point < 0:
                        sum_count += 1
                        sum += pow(global_point - local_point,2)
            if sum_count == 0:
                rmse = sys.float_info.max
            else:
                rmse = math.sqrt(sum/sum_count)
            # print("rmse="+str(rmse))
            if rmse < best_rmse:
                best_rmse = rmse
                best_index = i
            if i == int(len(self.poses_maps)/2):
                mid_rmse = rmse

        if mid_rmse <= best_rmse:
            best_rmse = mid_rmse
            best_index = int(len(self.poses_maps)/2)

        print("best rmse=" + str(best_rmse))
        print("pose diff="+str(best_index))
        # Update best with best index found
        self.best_map = self.poses_maps[best_index]
        self.best_pose = self.poses[best_index]

        # print("best pose: "+str(self.best_pose))

    # Generate full range of poses in x,y,theta and corresponding local maps
    def pose_generation(self):

        # %error in position and rotation
        ori_error = 0.4
        abs_error = 0.2

        original_data = self.local_map_data
        for i in range(self.k_poses):
            for j in range(self.k_poses):
                for k in range(self.k_poses):
                    prev_obs = self.local_map_data[0]
                    posed_data = original_data
                    for l in range(self.n_observations):
                        # Normalise i, j, k from 0,K_poses to -1,1
                        if self.k_poses > 1:
                            normalise_i = (i - ((self.k_poses - 1) / 2.0)) / ((self.k_poses - 1) / 2.0)
                            normalise_j = (j - ((self.k_poses - 1) / 2.0)) / ((self.k_poses - 1) / 2.0)
                            normalise_k = (k - ((self.k_poses - 1) / 2.0)) / ((self.k_poses - 1) / 2.0)
                        else:
                            normalise_i = 0
                            normalise_j = 0
                            normalise_k = 0

                        # Difference between all prev daya points and current
                        diff = posed_data[l] - prev_obs

                        # Distance travelled since last data point
                        dist_diff = math.sqrt(pow(diff[0], 2)+pow(diff[1], 2))

                        posed_data[l, 0] += (normalise_i * abs_error * diff[0])
                        posed_data[l, 1] += (normalise_j * abs_error * diff[1])
                        posed_data[l, 2] += (normalise_k * ori_error * diff[2])

                        prev_obs = posed_data[l]
                        if l == (self.n_observations - 1):
                            self.poses[((i*pow(self.k_poses,2))+(j*self.k_poses)+(k)),0] = posed_data[l,0]
                            self.poses[((i * pow(self.k_poses, 2)) + (j * self.k_poses) + (k)), 1] = posed_data[l, 1]
                            self.poses[((i * pow(self.k_poses, 2)) + (j * self.k_poses) + (k)), 2] = posed_data[l, 2]
                    self.poses_maps[(i*pow(self.k_poses,2))+(j*self.k_poses)+(k)] = self.create_local_map(posed_data)
                    # print("map created..")

        #print(self.poses_maps)

        return
    # Accumulate odom and boundary data over time
    def accum_data(self, x, y, ori):

        x -= self.start_x
        y -= self.start_y

        # Distance to the boundary, from the center point of the rear sensors
        boundary_dist = trend_function((self.rear_right.value + self.rear_left.value) / 2)

        # Calc difference in the distance of rear sensor avg and front sensor
        diff = ((trend_function(self.rear_left.value) + trend_function(self.rear_right.value)) / 2) - trend_function(
            self.front_center.value)

        # Hypotenuse of triangle made by front sensor, avg of rear sensors and the difference between their distance to the boundary
        hypot = 0.4657

        # Calculate angle to the boundary
        theta = math.degrees(math.acos(clean_cos(diff / hypot)))

        # Correct sign of angle based on whether it boundary is on the left or right
        if self.rear_right.value > self.rear_left.value:
            theta *= -1
        elif self.rear_right.value == self.rear_left.value:
            if self.front_center.value >= self.rear_left.value:
                theta = 0
            else:
                theta = 180

        self.local_map_data[self.local_map_index] = [x, y, ori, boundary_dist, theta]
        self.local_map_index += 1
        print(self.local_map_data)

    # Create a local map from a series of odom and boundary data
    def create_local_map(self, poses):

        local_grid = self.occupancy_grid
        # Iterate over all data points
        for a in range(len(poses)):
            x = poses[a, 0]
            y = poses[a, 1]
            ori = poses[a, 2]
            boundary_dist = poses[a, 3]
            theta = poses[a, 4]
            # Transform x,y to grid co-ordinates
            x_grid = self.origin_x + math.floor(self.grid_resolution * x)
            y_grid = self.origin_y + math.floor(self.grid_resolution * y)

            # Adjust boundary dist into occupancy grid frame of reference
            boundary_range = int(math.floor(self.grid_resolution * boundary_dist))

            draw_boundary = True
            if boundary_dist > 4:
                draw_boundary = False

            for i in range(2 * boundary_range):
                for j in range(2 * boundary_range):
                    # Distance from a given point in i and j from Automower
                    point_dist = math.sqrt(pow(i - boundary_range, 2) + pow(j - boundary_range, 2))
                    if point_dist < boundary_range:
                        tempx = int(i - boundary_range + x_grid)
                        tempy = int(j - boundary_range + y_grid)
                        map_point = local_grid[tempy, tempx]
                        if 0 <= map_point < self.green:
                            local_grid[tempy, tempx] = self.green - (self.certainty_max*(point_dist/boundary_range))

            local_grid[int(y_grid), int(x_grid)] = self.blue

            max_angle_err = 10*boundary_dist
            vari = int(10*boundary_dist)

            if draw_boundary:
                for i in range(vari):
                    norm_i = (i - ((vari - 1) / 2.0)) / ((vari - 1) / 2.0)
                    added_err = (norm_i * max_angle_err)
                    boundx = int(x_grid + boundary_range * math.cos(math.radians(correct_angle(theta + ori + added_err))))
                    boundy = int(y_grid + boundary_range * math.sin(math.radians(correct_angle(theta + ori + added_err))))
                    local_grid[boundy, boundx] = self.red + abs(added_err/max_angle_err)

        return local_grid

    def parse_loop(self, data):

        current_time = data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)

        # Calculate average of sensor readings over 1/update rate time window
        if self.interval_time == 0:
            self.interval_time = current_time
            self.observation_count = 0
            self.front_center.accum = 0
            self.front_right.accum = 0
            self.rear_left.accum = 0
            self.rear_right.accum = 0

        if current_time <= self.interval_time + (1.0/self.update_rate):
            self.observation_count += 1
            self.front_center.accum += data.frontCenter
            self.front_right.accum += data.frontRight
            self.rear_left.accum += data.rearLeft
            self.rear_right.accum += data.rearRight
        else:
            self.front_center.value = self.front_center.accum/self.observation_count
            self.front_right.value = self.front_right.accum/self.observation_count
            self.rear_left.value = self.rear_left.accum/self.observation_count
            self.rear_right.value = self.rear_right.accum/self.observation_count
            self.interval_time = 0

        # Plot map (local/global) on a certain time interval
        if self.plot_timer == 0:
            self.plot_timer = current_time

        if current_time >= self.plot_timer + self.plot_interval:
            print("PRINTING")
            self.plot_timer = 0
            ##
            plt.cla()
            plt.clf()
            ##
            # tell imshow about color map so that only set colors are used
            img = plt.imshow(self.occupancy_grid_global, interpolation='nearest', origin='lower', cmap='RdYlGn')
            # make a color bar
            plt.colorbar(img, cmap=self.cmap, norm=self.norm, boundaries=self.bounds, ticks=[-1,0,1])
            # plt.show()
            plt.savefig("map" + '.png')

        if self.start_time is None:
            self.start_time = data.header.stamp.secs
        if data.header.stamp.secs > (self.start_time + self.time_out) and self.time_out > 0:
            rospy.signal_shutdown("time out")

        # Are any negative
        if self.front_center.value < 0 or self.front_right.value < 0 or self.rear_left.value < 0 or self.rear_right.value < 0:
            return
        else:
            return

    def wheel_encoder(self, data):
        if not (self.prev_tick is None):
            # Accumulate encoder data
            self.wheel_l_accum += (data.header.seq - self.prev_tick) * data.lwheel
            self.wheel_r_accum += (data.header.seq - self.prev_tick) * data.rwheel
            # Update time variable
            self.time = (data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)) - self.start_time

        else:
            # Record start time, this is only called on the first iteration
            self.start_time = data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)

        # Record previous tick
        self.prev_tick = data.header.seq

    def run(self):
        try:
            # Setup subscribers
            rospy.Subscriber("loop", Loop, self.parse_loop, queue_size=None)
            rospy.Subscriber("wheel_encoder", WheelEncoder, self.wheel_encoder, queue_size=None)
            r = rospy.Rate(self.update_rate)
            while not rospy.is_shutdown():
                r.sleep()
                self.update()
        except rospy.exceptions.ROSInterruptException:
            pass
        finally:
            self.fini()

    def fini(self):
        print("Shutting down")


def correct_angle(ang):
    if ang > 180:
        return ang - 360
    elif ang < -180:
        return 360 + ang
    else:
        return ang


def odometry(e_left, e_right, x_start, y_start, theta_start):
    wheel_separation_distance = 0.48

    delta_theta = (e_right - e_left) / wheel_separation_distance
    delta_s = (e_right + e_left) / 2
    x = x_start + (delta_s * math.cos(math.radians(theta_start) + (delta_theta / 2)))
    y = y_start + (delta_s * math.sin(math.radians(theta_start) + (delta_theta / 2)))
    theta = theta_start + math.degrees(delta_theta)
    return x, y, correct_angle(theta)


# Function to take quarternion of automower orientation in 3D space and convert to an angle from a bird eye view
def quarternion_to_angle(x, y, z, w):
    ysqr = y * y

    t1 = +2.0 * (w * z + x * y)
    t2 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t1, t2)

    return Z


def clean_cos(cos_angle):
    return min(1,max(cos_angle,-1))


def trend_function(x):
    # a = 8610.6
    # b = 0.443

    min = 3583
    a = pow(2, 14) - min  # 16500~~~ - min
    b = 0.667

    if x < 0:
        min *= -1

    if x == 0:
        return 0

    val = np.sign(x) * (1 / b) * np.log( (1.0*a) / abs(x - min))
    return val

    #return np.sign(x)*pow(a/abs(x), 1/b)


if __name__ == '__main__':
    rospy.init_node('slam_sim')
    slam_sim = SlamSim()
    slam_sim.run()
