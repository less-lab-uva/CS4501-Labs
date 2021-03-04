#!/usr/bin/env python
import rospy
import math
import os
import copy
import numpy as np
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan


class LineSegment:

    def __init__(self, a, b, c, d):
        self.start_point = np.array([a, b])
        self.end_point = np.array([c, d])
        self.point_difference = np.subtract(self.end_point, self.start_point)

    def __repr__(self):
        return '(%d, %d, %d, %d)' % (self.start_point[0], self.start_point[1], self.end_point[0], self.end_point[1])


# Create a class which saves the location of the drone and simulates the lidar
class Lidar:

    # Node initialization
    def __init__(self):

        # Get the default path (need to remove src)
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.dir_path = self.dir_path[:-3] + "data/"

        # Get image name and load it from data folder
        self.file_path = rospy.get_param('/lidar_node/map_path', "room.txt")

        self.angle_min = float(rospy.get_param('/lidar_node/angle_min', str(-math.pi)))  # from -180
        self.angle_max = float(rospy.get_param('/lidar_node/angle_max', str(math.pi)))  # to +180
        self.reading_count = int(rospy.get_param('/lidar_node/angle_max', str(360)))  # 1 degree resolution
        self.angle_increment = (self.angle_max - self.angle_min) / self.reading_count

        self.range_max = float(rospy.get_param('/lidar_node/range_max', str(30)))

        self.range_noise = rospy.get_param('/uav/sensors/lidar_range_noise', str(0.5))

        self.lidar_position_noise = rospy.get_param('/uav/sensors/lidar_position_noise', str(0.5))

        self.walls = []
        with open(self.dir_path + self.file_path, 'r') as f:
            for line in f.readlines():
                (a, b), (c, d) = eval(line)
                self.walls.append(LineSegment(a, b, c, d))

        # Create the subscribers
        self.position_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.getPosition, queue_size=1)

        # Create the publishers
        self.lidar_pub = rospy.Publisher("/uav/sensors/lidar", LaserScan, queue_size=10)
        self.lidar_position_pub = rospy.Publisher("/uav/sensors/lidar_position", PointStamped, queue_size=10)

        # Save the drones positions
        self.position = np.zeros(3, dtype=np.float64)
        self.yaw = 0

        # Call the mainloop of our class
        self.mainloop()

    # Callback for the keyboard manager
    def getPosition(self, msg):
        # Save the drones alitude
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        self.position[2] = msg.pose.position.z

        # Get the drones attitude
        quarternion_pose = (msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z,
                            msg.pose.orientation.w)
        self.yaw = np.array(euler_from_quaternion(quarternion_pose))[2]
        
        self.noisy_position = PointStamped()
        self.noisy_position.header = copy.deepcopy(msg.header)
        self.noisy_position.point.x = np.random.normal(msg.pose.position.x, self.lidar_position_noise)
        self.noisy_position.point.y = np.random.normal(msg.pose.position.y, self.lidar_position_noise)
        self.noisy_position.point.z = np.random.normal(msg.pose.position.z, self.lidar_position_noise)
        self.lidar_position_pub.publish(self.noisy_position)

    # The main loop of the function
    def mainloop(self):
        # Set the rate of this loop
        rate_hz = 10
        rate_sec = 1/rate_hz
        rate = rospy.Rate(rate_hz)

        # While ROS is still running
        while not rospy.is_shutdown():
            try:
                position = np.array([self.position[0], self.position[1]])
                scan = LaserScan()
                scan.header.stamp = rospy.Time.now()
                # TODO for rviz to be able to show the scan, we need to publish a transform into the odom frame
                scan.header.frame_id = "uav/imu_level"
                scan.angle_min = self.angle_min
                scan.angle_max = self.angle_max
                scan.angle_increment = self.angle_increment
                scan.scan_time = rate_sec  # unused
                scan.intensities = []  # unused
                scan.range_min = 0
                scan.range_max = self.range_max
                scan.ranges = np.zeros(self.reading_count, dtype=np.float32)
                # simulate the ray tracing for the lidar to the simulated walls
                # adapted from https://stackoverflow.com/questions/14307158/how-do-you-check-for-intersection-between-a-line-segment-and-a-line-ray-emanatin
                for reading in range(self.reading_count):
                    min_dist = np.inf
                    angle = self.yaw + self.angle_min + reading * self.angle_increment
                    x = math.cos(angle)
                    y = math.sin(angle)
                    v3 = np.array([-y, x])
                    for wall in self.walls:
                        v1 = np.subtract(position, wall.start_point)
                        dot = np.dot(wall.point_difference, v3)
                        # if the ray and the line are parallel, they cannot intersect
                        if abs(dot) < 1e-6:
                            continue  # the value is infinite, so we do not need to update anything
                        dist = np.cross(wall.point_difference, v1) / dot
                        intersection_point = np.dot(v1, v3) / dot
                        # if the distance along the ray is positive and the intersection is between the start and end, check the distance
                        if dist >= 0.0 and 0.0 <= intersection_point <= 1.0:
                            min_dist = min(min_dist, dist)
                    # if the value is beyond our maximum range, return inf
                    scan.ranges[reading] = np.inf if min_dist > self.range_max else min_dist
                # Add noise
                scan.ranges += np.random.normal(0, self.range_noise, scan.ranges.shape)
                # Publish
                self.lidar_pub.publish(scan)
            # Print error
            except Exception as e:
                print e

            # Sleep for the remainder of the loop
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('lidar_node')
    try:
        ktp = Lidar()
    except rospy.ROSInterruptException:
        pass
