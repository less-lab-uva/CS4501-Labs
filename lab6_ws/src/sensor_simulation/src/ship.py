#!/usr/bin/env python
import os, time
import rospy
import math
import os
import copy
import numpy as np
import random
from tf.transformations import euler_from_quaternion
from threading import Lock
from wpgen import WpGen
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, PoseStamped, Vector3Stamped


class Ship:

    def __init__(self):
        self.waypoints = None
        self.ship_pose = None
        self.uav_pose = None
        self.beacon = None
        self.wp_generator = None
        self.epsilon_followed = rospy.get_param(rospy.get_name() + "/followed_epsilon", 0.2)
        self.epsilon_beacon2 = 2.5
        self.beacon_height_thresh = 9
        self.velocity = rospy.get_param(rospy.get_name() + "/ship_velocity",
                                        0.2)  # units/s, 0.2 is slowest option because 0.1 takes forever to gen images
        self.hz = 4
        self.lock = Lock()
        self.beacon1_publisher = rospy.Publisher('/ship/beacon1', Vector3Stamped, queue_size=1)
        self.beacon2_publisher = rospy.Publisher('/ship/beacon2', Vector3Stamped, queue_size=1)
        self.followed_publisher = rospy.Publisher('/ship/followed', Bool, queue_size=1)
        self.uav_subscriber = rospy.Subscriber("/uav/sensors/gps", PoseStamped, self.get_gps)
        # set up ship trajectory and images
        self.process_waypoints_string()
        self.setup_trajectory()
        self.mainloop()

    def process_waypoints_string(self):
        self.lock.acquire()
        waypoints_str = rospy.get_param(rospy.get_name() + "/ship_waypoints", "[[5,0],[-2,-5],[-5,0],[0,5]]")
        # Convert from string to numpy array
        self.waypoints = waypoints_str.replace('[', '')
        self.waypoints = self.waypoints.replace(']', '')
        self.waypoints = np.fromstring(self.waypoints, dtype=int, sep=',')
        self.waypoints = np.reshape(self.waypoints, (-1, 2))
        self.lock.release()

    # Velocity between 0.0 and 5.0 units/s in increments of 0.2
    def setup_trajectory(self):
        self.lock.acquire()
        # self.velocity = rospy.get_param(rospy.get_name() + "/ship_velocity", 0.2)
        self.wp_generator = WpGen(self.waypoints, self.velocity, self.hz)
        self.waypoints = self.wp_generator.setup_trajectory_from_waypoints()
        self.lock.release()

    def process_waypoints_file(self, filename):
        with open(filename, 'r') as f:
            lines = f.readlines()
            self.waypoints = []  # np.empty((len(lines),6))
            for i, line in enumerate(lines):
                # Convert from string to numpy array
                try:
                    line = line.replace('[', '')
                    line = line.replace(']\n', '')
                    line = np.fromstring(line, sep=',')
                    self.waypoints.append(line)
                except Exception as e:
                    print("Error: Could not convert ship waypoints to Numpy array")
                    print(line)
                    print(e)
                    exit()
        self.waypoints = np.array(self.waypoints)

    def get_gps(self, msg):
        self.uav_pose = msg

    def get_true_distance(self):
        # self.lock.acquire()
        if self.uav_pose:
            x_dist = self.ship_pose[0] - self.uav_pose.pose.position.x
            y_dist = self.ship_pose[1] - self.uav_pose.pose.position.y
            return math.sqrt(math.pow(x_dist, 2) + math.pow(y_dist, 2))
        else:
            return float('inf')
            self.beacon1 = None
        # self.lock.release()

    def populate_beacon1_msg(self):
        self.lock.acquire()
        self.beacon1 = Vector3Stamped()
        self.beacon1.header.stamp = rospy.Time.now()
        if self.uav_pose:
            if self.uav_pose.pose.position.z < self.beacon_height_thresh:
                self.beacon1 = None
                rospy.loginfo("Drone is outside altitude range for beacon1 to send message")
            else:
                self.beacon1.vector.x = 1.33 * (self.ship_pose[0] - self.uav_pose.pose.position.x) + random.gauss(-0.5,
                                                                                                                  0.5)
                self.beacon1.vector.y = 1.33 * (self.ship_pose[1] - self.uav_pose.pose.position.y) + random.gauss(-0.5,
                                                                                                                  0.5)
                self.beacon1.vector.z = 0
        else:
            rospy.loginfo("Cannot populate beacon message")
            self.beacon1 = None
        self.lock.release()

    def populate_beacon2_msg(self):
        self.lock.acquire()
        self.beacon2 = Vector3Stamped()
        self.beacon2.header.stamp = rospy.Time.now()
        if self.uav_pose:
            if self.uav_pose.pose.position.z < self.beacon_height_thresh:
                self.beacon2 = None
                rospy.loginfo("Drone is outside altitude range for beacon2 to send message")
            else:
                self.beacon2.vector.x = (self.ship_pose[0] - self.uav_pose.pose.position.x) * 3
                self.beacon2.vector.y = (self.ship_pose[1] - self.uav_pose.pose.position.y) * 3
                self.beacon2.vector.z = 0
        else:
            rospy.loginfo("Cannot populate beacon message")
            self.beacon2 = None
        self.lock.release()

    def mainloop(self):
        rate = rospy.Rate(4)
        i = 0
        while not rospy.is_shutdown():
            self.ship_pose = self.waypoints[i]
            rospy.loginfo("Ship pose: {}".format(self.ship_pose, prec=1))

            if self.uav_pose and self.uav_pose.pose.position.z >= self.beacon_height_thresh:
                rospy.loginfo(
                    "UAV pose: [{}, {}, {}]".format(self.uav_pose.pose.position.x, self.uav_pose.pose.position.y,
                                                    self.uav_pose.pose.position.z, prec=1))

                # Publish beacon1 if UAV above 9m
                self.populate_beacon1_msg()
                if self.beacon1:
                    rospy.loginfo("beacon1: [{}, {}, {}]".format(self.beacon1.vector.x, self.beacon1.vector.y,
                                                                 self.beacon1.vector.z, prec=1))
                    self.beacon1_publisher.publish(self.beacon1)
                else:
                    rospy.loginfo("beacon1: None")

                dist = self.get_true_distance()
                rospy.loginfo("True distance: " + str(round(dist, 1)))

                if dist and dist < self.epsilon_beacon2:
                    self.populate_beacon2_msg()
                    if self.beacon2:
                        rospy.loginfo(
                            "beacon2: [{}, {}, {}]".format(self.beacon2.vector.x, self.beacon2.vector.y,
                                                           self.beacon2.vector.z,
                                                           prec=1))
                        self.beacon2_publisher.publish(self.beacon2)
                    else:
                        rospy.loginfo("beacon2: None")

                if dist and dist < self.epsilon_followed:
                    rospy.loginfo("Drone sufficiently close to the ship!")
                    self.followed_publisher.publish(True)
                    rospy.loginfo("Followed: True")
                else:
                    self.followed_publisher.publish(False)
                    rospy.loginfo("Followed: False")

            else:
                # rospy.loginfo("Drone too low; Beacon connectivity unavailable")
                rospy.loginfo("self.uav_pose is None")
            i = (i + 1) % len(self.waypoints)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ship_node')
    try:
        s = Ship()
    except rospy.ROSInterruptException:
        pass
