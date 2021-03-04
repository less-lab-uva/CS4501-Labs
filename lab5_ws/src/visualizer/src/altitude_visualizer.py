#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseStamped
from altitude.msg import AltitudeStamped
from visualization_msgs.msg import Marker


class AltitudeVisualizer:
    # Node initialization
    def __init__(self):
        # Create the publisher and subscriber
        self.marker_pub = rospy.Publisher('/uav/visualizer/altitude_marker',
                                   Marker,
                                   queue_size=1)

        self.sub = rospy.Subscriber('/uav/sensors/altitude',
                                    AltitudeStamped, self.process_altitude,
                                    queue_size=1)

        self.sub = rospy.Subscriber('/uav/sensors/altitude_ma',
                                    AltitudeStamped, self.process_altitude_ma,
                                    queue_size=1)

        self.sub = rospy.Subscriber('/uav/sensors/altitude_fused_ma',
                                    AltitudeStamped, self.process_altitude_fused_ma,
                                    queue_size=1)

        self.sub = rospy.Subscriber('/uav/sensors/altitude_kalman',
                                    AltitudeStamped, self.process_altitude_kalman,
                                    queue_size=1)
        rospy.spin()

    def process_altitude(self, msg):
        self.generate_marker(msg, 0, (1, 1, 0))  # yellow

    def process_altitude_ma(self, msg):
        self.generate_marker(msg, 1, (0, 1, 0))  # green

    def process_altitude_fused_ma(self, msg):
        self.generate_marker(msg, 2, (1, 0, 0))  # red

    def process_altitude_kalman(self, msg):
        self.generate_marker(msg, 3, (1, 0, 1))  # magenta

    def generate_marker(self, msg, id, color):
        marker = Marker()
        marker.header.frame_id = "uav/imu_ground"
        marker.header.stamp = msg.stamp
        marker.ns = "uav"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = msg.value
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = msg.error
        marker.scale.y = msg.error
        marker.scale.z = msg.error
        marker.color.a = 0.5
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        self.marker_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('altitude_visualization_node')
    try:
        AltitudeVisualizer()
    except rospy.ROSInterruptException:
        pass