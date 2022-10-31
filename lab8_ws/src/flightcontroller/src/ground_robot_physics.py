#!/usr/bin/env python
import rospy
import tf2_ros
import time
import copy
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_point


class GroundRobotPhysics:

    def __init__(self):
        time.sleep(10)
        # Used by the callback for the topic /tower/goal
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'world'
        self.goal = None
        self.max_speed = 0.5
        self.goal_sub = rospy.Subscriber("/ground_robot/goal", Vector3, self.get_goal)
        self.gps_pub = rospy.Publisher("/ground_robot/gps", PoseStamped, queue_size=1)
        # start main loop
        self.mainloop()

    def get_goal(self, msg):
        self.goal = copy.copy(msg)

    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.goal is not None:
                dx = self.goal.x - self.pose.pose.position.x
                dy = self.goal.y - self.pose.pose.position.y
                angle = math.atan2(dy, dx)
                # normalize so that we move either to the goal or at max speed, whichever is less
                to_move_x = self.max_speed * math.cos(angle)
                to_move_y = self.max_speed * math.sin(angle)
                if abs(dx) < abs(to_move_x):
                    to_move_x = dx
                if abs(dy) < abs(to_move_y):
                    to_move_y = dy
                self.pose.pose.position.x += to_move_x
                self.pose.pose.position.y += to_move_y
                self.pose.header.stamp = rospy.Time.now()
                # the pose pub is locked behind the goal being set because we don't want the robot to be visible until it has a goal
                self.gps_pub.publish(self.pose)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ground_robot_physics')
    try:
        tom = GroundRobotPhysics()
    except rospy.ROSInterruptException:
        pass
