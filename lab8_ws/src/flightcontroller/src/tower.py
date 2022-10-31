#!/usr/bin/env python
import rospy
import time
import copy

import numpy as np

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

import traceback
class Tower:

    def __init__(self):

        time.sleep(10)
        rospy.loginfo("Initializing tower")

        # Goal publisher
        self.goal_pub = rospy.Publisher("/tower/goal", Vector3, queue_size=1)

        # map variables
        self.map = None
        self.map_origin = None
        self.empty_traj = True

        # subscribers subscriber
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)
        self.traj_sub = rospy.Subscriber('/uav/trajectory', Int32MultiArray, self.handle_traj)

        # Transform
        # This is the transfrom from tester node
        self.towertransform = TransformStamped()
        self.towertransform.transform.translation.x = float(-128)
        self.towertransform.transform.translation.y = float(77)
        self.towertransform.transform.translation.z = float(0)

        quat = quaternion_from_euler(float(0), float(0), float(+0.523599))
        self.towertransform.transform.rotation.x = quat[0]
        self.towertransform.transform.rotation.y = quat[1]
        self.towertransform.transform.rotation.z = quat[2]
        self.towertransform.transform.rotation.w = quat[3]

        self.towertransform.header.frame_id = 'tower'
        self.towertransform.child_frame_id = 'world'

        # start main loop
        self.mainloop()

    def handle_traj(self, msg):
        if len(msg.data) <= 2:
            self.empty_traj = True
        else:
            self.empty_traj = False

    # Map callback
    def get_map(self, msg):
        # Get map transform
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        # Get the map
        self.map = np.reshape(msg.data, (msg.info.width, msg.info.height))
        self.map = self.expand_obstacles(self.map, 1)

    def generate_goal(self):
        try:
            MAX_COUNT = self.map.shape[0] * self.map.shape[1]
            count = 0
            while count < MAX_COUNT:
                # choose a random position
                i = np.random.randint(0, self.map.shape[0])
                j = np.random.randint(0, self.map.shape[1])
                if self.map[i][j] == 0:
                    return [i + self.map_origin[0], j + self.map_origin[1]]
                count += 1
            return None
        except:
            traceback.print_exc()
            return None

    def mainloop(self):

        # Set the rate of this loop
        rate = rospy.Rate(0.1)
        rate.sleep()

        while not rospy.is_shutdown():
            if self.empty_traj:
                new_goal = self.generate_goal()
                if new_goal:
                    point = PointStamped()
                    point.point.x = new_goal[0]
                    point.point.y = new_goal[1]
                    point.point.z = 0.0
                    new_point = do_transform_point(point, self.towertransform)
                    rospy.loginfo(str(rospy.get_name()) + ": Published goal {}".format([new_point.point.x, new_point.point.y]))
                    msg = Vector3(new_point.point.x, new_point.point.y, 0)
                    self.goal_pub.publish(msg)

            # Sleep for the remainder of the loop
            rate.sleep()

    # Expand the obstacles by distance so you do not hit one
    def expand_obstacles(self, map_data, distance):
        new_map = copy.deepcopy(map_data)
        # For each element in the map
        for i in range(map_data.shape[0]):
            for j in range(map_data.shape[1]):
                # if this is an obstacle
                if map_data[i, j] != 0:
                    # Expand the obstacle by 1 in all directions
                    for x_dim in range((-1 * distance), (1 * distance) + 1):
                        for y_dim in range((-1 * distance), (1 * distance) + 1):
                            if (0 <= i + x_dim < new_map.shape[0]) and (0 <= j + y_dim < new_map.shape[1]):
                                new_map[i + x_dim, j + y_dim] = map_data[i, j]
        return new_map


if __name__ == '__main__':
    rospy.init_node('tower')
    try:
        pp = Tower()
    except rospy.ROSInterruptException:
        pass
