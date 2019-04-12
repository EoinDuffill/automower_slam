#!/usr/bin/env python
import rospy
import numpy as np
import math
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


class BoundaryCalcs(object):

    def __init__(self):

        # FC, FR, RL, RR
        self.front_center = BoundarySensor(0.3, 0)
        self.front_right = BoundarySensor(0.3, -0.07)
        self.rear_left = BoundarySensor(-0.15, 0.12)
        self.rear_right = BoundarySensor(-0.15, -0.12)
        self.observation_count = 0
        self.boundary_interval_time = 0

        self.file = open("results_distance.txt", "w")

        self.update_rate = 5

        self.prev_tick = None
        self.start_time = 0
        self.time = 0
        self.wheel_l_accum = 0
        self.wheel_r_accum = 0

        self.time_init = 0
        self.dist = 0
        self.dist_init = 0
        self.interval_time = 1
        self.state = 0

        self.x = 0
        self.init_x = 0
        self.y = 0
        self.init_y = 0
        self.orientation = 0

    def update(self):

        # SIM
        # Get initial variables from simulator
        data = rospy.wait_for_message("gazebo/model_states", ModelStates)
        # get the index into the data for the automower
        index = data.name.index("automower")
        # initial x,y #SIM
        self.x = data.pose[index].position.x
        self.y = data.pose[index].position.y
        # initial angle, converted from a quarternion #SIM
        self.orientation = quarternion_to_angle(data.pose[index].orientation.x, data.pose[index].orientation.y,
                                                     data.pose[index].orientation.z, data.pose[index].orientation.w)

        self.dist = trend_function(self.front_center.value)
        dist2 = (trend_function(self.rear_left.value) + trend_function(self.rear_right.value))/2

        # Hypotenuse of triangle made by front sensor, avg of rear sensors and the difference between their distance to the boundary
        hypot = 0.4657

        # Calculate angle to the boundary
        theta = math.degrees(math.acos(clean_cos((dist2 - self.dist) / hypot)))

        # Correct sign of angle based on whether it boundary is on the left or right
        if self.rear_right.value > self.rear_left.value:
            theta *= -1
        elif self.rear_right.value == self.rear_left.value:
            if self.front_center.value >= self.rear_left.value:
                theta = 0
            else:
                theta = 180

        if self.state == 0:
            self.dist_init = self.dist
            self.time_init = self.time
            self.init_x = self.x
            self.init_y = self.y
            self.state = 1
            self.wheel_r_accum = 0
            self.wheel_l_accum = 0
        elif self.state == 1:
            if self.time > self.time_init + self.interval_time:
                self.state = 2
        elif self.state == 2:
            dist_final = self.dist
            (x, y, delta_theta) = odometry(self.wheel_l_accum, self.wheel_r_accum, 0, 0, 0)
            hypot2 = math.sqrt(pow(x, 2)+pow(y, 2))
            if hypot2 > 0.15 and delta_theta < 10:
                theta2 = math.degrees(math.acos(clean_cos((self.dist_init - dist_final) / hypot2)))
                #print("Debug, pre acos:"+str(clean_cos((self.dist_init - dist_final) / hypot2))+", post acos"+str(math.acos(clean_cos((self.dist_init - dist_final) / hypot2))))
                #print("to degrees:"+str(math.degrees(math.acos(clean_cos((self.dist_init - dist_final) / hypot2)))))
                print("actual dist:"+str(self.y + 0.125 + (0.38* math.cos(math.radians(180 + self.orientation))))+", ang:"+str(180 + math.degrees(self.orientation)))
                print("front dist :" + str(self.dist) + ", angle:" + str(theta))
                #print("rear diff:"+str(abs(trend_function(self.rear_left.value) - trend_function(self.rear_right.value))))
                # print("front dist2:" + str(dist_final) + ", angle:" + str(theta2))
                #print("hypot:"+str(hypot2)+"actual hypot:"+str(math.sqrt(pow(self.x - self.init_x, 2)+pow(self.y - self.init_y, 2))))
                print("val init:"+str(self.dist_init)+", val curr:"+str(dist_final)+"error to hypot"+str(abs(self.dist_init - dist_final) - hypot2))
            self.state = 0


    def fini(self):
        print("Shutting down")

    def parse_loop(self, data):

        current_time = data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)

        # Calculate average of sensor readings over 1/update rate time window
        if self.boundary_interval_time == 0:
            self.boundary_interval_time = current_time
            self.observation_count = 0
            self.front_center.accum = 0
            self.front_right.accum = 0
            self.rear_left.accum = 0
            self.rear_right.accum = 0

        if current_time <= self.boundary_interval_time + (1.0 / (10*self.update_rate)):
            self.observation_count += 1
            self.front_center.accum += data.frontCenter
            self.front_right.accum += data.frontRight
            self.rear_left.accum += data.rearLeft
            self.rear_right.accum += data.rearRight
        else:
            self.front_center.value = (self.front_center.accum*1.0) / self.observation_count
            self.front_right.value = (self.front_right.accum*1.0) / self.observation_count
            self.rear_left.value = (self.rear_left.accum*1.0) / self.observation_count
            self.rear_right.value = (self.rear_right.accum*1.0) / self.observation_count
            self.boundary_interval_time = 0

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

    min = 3649
    a = 10578  # 16500~~~ - min
    b = 0.643

    if x < 0:
        min *= -1

    if x == 0:
        return 0

    val = np.sign(x) * (1 / b) * np.log( (1.0*a) / abs(x - min))
    return val + 0.12

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


if __name__ == '__main__':
    rospy.init_node('boundary_calculations')
    calcs = BoundaryCalcs()
    calcs.run()


