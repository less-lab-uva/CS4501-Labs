#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Quaternion
import numpy as np
import copy
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# Create a class which saves the altitude of the drone and esimates the pressure
class NoiseAndTransformationsNode:

    # Node initialization
    def __init__(self):
        rospy.init_node('noise_and_transformations_node')
        # Sleep to allow simulation to start
        time.sleep(5)

        # Create the publisher and subscriber

        self.position_noise_lvl = float(rospy.get_param('/uav/sensors/position_noise_lvl', str(0))) / 3
        self.attitude_noise_lvl = float(rospy.get_param('/uav/sensors/attitude_noise_lvl', str(0))) / 3
        self.velocity_noise_lvl = float(rospy.get_param('/uav/sensors/velocity_noise_lvl', str(0))) / 3
        self.velocity_attitude_noise_lvl = float(rospy.get_param('/uav/sensors/velocity_attitude_noise_lvl', str(0))) / 3
        self.tf_pub = tf2_ros.TransformBroadcaster()

        self.position = PoseStamped()
        self.ground_truth = PoseStamped()
        self.ground_truth_velocity = TwistStamped()
        self.velocity = TwistStamped()
        self.gps_noise_pub = rospy.Publisher("/uav/sensors/gps_noisy", PoseStamped, queue_size=1)
        self.ground_truth_pub = rospy.Publisher("/uav/sensors/grount_truth", PoseStamped, queue_size=1)
        self.velocity_noise_pub = rospy.Publisher("/uav/sensors/velocity_noisy", TwistStamped, queue_size=1)

        self.position_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.get_position, queue_size=1)
        self.velocity_sub = rospy.Subscriber('/uav/sensors/velocity', TwistStamped, self.get_velocity, queue_size=1)
        rospy.spin()

    def get_velocity(self, msg):
        self.ground_truth_velocity = msg
        self.velocity = copy.deepcopy(msg)
        self.velocity.twist.linear.x += np.random.normal(0, self.velocity_noise_lvl)
        self.velocity.twist.linear.y += np.random.normal(0, self.velocity_noise_lvl)
        self.velocity.twist.linear.z += np.random.normal(0, self.velocity_noise_lvl)

        self.velocity.twist.angular.x += np.random.normal(0, self.velocity_attitude_noise_lvl)
        self.velocity.twist.angular.y += np.random.normal(0, self.velocity_attitude_noise_lvl)
        self.velocity.twist.angular.z += np.random.normal(0, self.velocity_attitude_noise_lvl)

        self.velocity_noise_pub.publish(self.velocity)

    def get_position(self, msg):
        self.ground_truth = msg
        self.ground_truth_pub.publish(self.ground_truth)
        self.position = copy.deepcopy(msg)
        self.position.pose.position.x += np.random.normal(0, self.position_noise_lvl)
        self.position.pose.position.y += np.random.normal(0, self.position_noise_lvl)
        self.position.pose.position.z += np.random.normal(0, self.position_noise_lvl)
        quat = self.position.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        roll_noisy = np.random.normal(roll, self.attitude_noise_lvl)
        pitch_noisy = np.random.normal(pitch, self.attitude_noise_lvl)
        yaw_noisy = np.random.normal(yaw, self.attitude_noise_lvl)
        self.position.pose.orientation = Quaternion(*quaternion_from_euler(roll_noisy, pitch_noisy, yaw_noisy))
        self.gps_noise_pub.publish(self.position)
        imu_level = TransformStamped()
        imu_level.header.frame_id = "world"
        imu_level.child_frame_id = "uav/imu_level"
        imu_level.header.stamp = rospy.Time.now()
        imu_level.transform.translation.x = self.ground_truth.pose.position.x
        imu_level.transform.translation.y = self.ground_truth.pose.position.y
        imu_level.transform.translation.z = self.ground_truth.pose.position.z
        imu_level.transform.rotation = Quaternion(*quaternion_from_euler(0, 0, yaw))
        self.tf_pub.sendTransform(imu_level)
        imu_ground = TransformStamped()
        imu_ground.header.frame_id = "world"
        imu_ground.child_frame_id = "uav/imu_ground"
        imu_ground.header.stamp = rospy.Time.now()
        imu_ground.transform.translation.x = self.ground_truth.pose.position.x
        imu_ground.transform.translation.y = self.ground_truth.pose.position.y
        imu_ground.transform.translation.z = 0
        imu_ground.transform.rotation = Quaternion(*quaternion_from_euler(0, 0, yaw))
        self.tf_pub.sendTransform(imu_ground)


if __name__ == '__main__':
    try:
        ktp = NoiseAndTransformationsNode()
    except rospy.ROSInterruptException:
        pass
