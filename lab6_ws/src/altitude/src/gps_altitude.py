#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseStamped
from altitude.msg import AltitudeStamped

class GPSAltitudeMA:
    # Node initialization
    def __init__(self):
        # Create the publisher and subscriber
        self.pub = rospy.Publisher('/uav/sensors/altitude',
                                   AltitudeStamped,
                                   queue_size=1)
        self.sub = rospy.Subscriber('/uav/sensors/gps_noisy',
                                    PoseStamped, self.process_altitude,
                                    queue_size=1)
        self.gps_error = float(rospy.get_param('/uav/sensors/position_noise_lvl', str(0)))
        self.altitude = AltitudeStamped()
        self.gps_pose = PoseStamped()
        rospy.spin()

    def process_altitude(self, msg):
        self.gps_pose = msg
        self.altitude.value = self.gps_pose.pose.position.z
        self.altitude.stamp = self.gps_pose.header.stamp
        self.altitude.error = self.gps_error
        self.pub.publish(self.altitude)


if __name__ == '__main__':
    rospy.init_node('gps_altitude_ma_node')
    try:
        GPSAltitudeMA()
    except rospy.ROSInterruptException:
        pass


