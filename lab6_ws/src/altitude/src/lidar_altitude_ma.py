#!/usr/bin/env python
import math

import rospy
import time
from geometry_msgs.msg import PointStamped
from altitude.msg import AltitudeStamped
from moving_average import MovingAverage

class LIDARAltitudeMA:
    # Node initialization
    def __init__(self):
        # Create the publisher and subscriber
        self.moving_average = MovingAverage(5)
        self.pub = rospy.Publisher('/uav/sensors/lidar_altitude_ma',
                                   AltitudeStamped,
                                   queue_size=1)
        self.sub = rospy.Subscriber('/uav/sensors/lidar_position',
                                    PointStamped, self.process_altitude,
                                    queue_size=1)
        self.lidar_error = float(rospy.get_param('/uav/sensors/lidar_position_noise', str(0)))
        self.altitude = AltitudeStamped()
        self.lidar_point = PointStamped()
        rospy.spin()

    def process_altitude(self, msg):
        self.lidar_point = msg
        self.moving_average.add(self.lidar_point.point.z)
        self.altitude.value = self.moving_average.get_average()
        self.altitude.stamp = self.lidar_point.header.stamp
        self.altitude.error = self.lidar_error / math.sqrt(self.moving_average.window_size)
        self.pub.publish(self.altitude)


if __name__ == '__main__':
    rospy.init_node('lidar_altitude_ma_node')
    try:
        LIDARAltitudeMA()
    except rospy.ROSInterruptException:
        pass


