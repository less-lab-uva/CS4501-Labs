#!/usr/bin/env python
import math

import rospy
from threading import Lock
from geometry_msgs.msg import TwistStamped
from altitude.msg import AltitudeStamped



class AltitudeKalman:

    # Node initialization
    def __init__(self):
        self.altitude = 0
        self.altitude_error = 0
        self.last_update = rospy.Time()
        self.velocity = 0
        self.lock = Lock()
        self.changed = False
        self.has_velocity_data = False
        self.predicted_altitude = None
        self.predicted_altitude_error = None
        self.velocity_error = float(rospy.get_param('/uav/sensors/velocity_noise_lvl', str(0)))
        # Create the publisher and subscriber
        self.pub = rospy.Publisher('/uav/sensors/altitude_kalman',
                                   AltitudeStamped,
                                   queue_size=1)
        self.altitude_sub = rospy.Subscriber('/uav/sensors/altitude',
                                    AltitudeStamped, self.process_altitude,
                                    queue_size=1)
        self.vel_sub = rospy.Subscriber('/uav/sensors/velocity_noisy',
                                       TwistStamped, self.process_velocity,
                                       queue_size=1)
        self.mainloop()

    def process_altitude(self, msg):
        self.lock.acquire()
        self.altitude = msg.value
        self.altitude_error = msg.error
        self.changed = True
        self.lock.release()


    def process_velocity(self, msg):
        self.lock.acquire()
        self.velocity = msg.twist.linear.z
        self.has_velocity_data = True
        self.lock.release()

    # The main loop of the function
    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(10)
        # While ROS is still running
        while not rospy.is_shutdown():
            self.lock.acquire()
            if self.changed and self.altitude_error > 0:
                if self.predicted_altitude is None:
                    self.predicted_altitude = self.altitude
                    self.predicted_altitude_error = self.altitude_error
                else:
                    predicted_error_2 = math.pow(self.predicted_altitude_error, 2)
                    fused_error_2 = math.pow(self.altitude_error, 2)
                    self.predicted_altitude = (predicted_error_2 * self.altitude + fused_error_2 * self.predicted_altitude) / (predicted_error_2 + fused_error_2)
                    self.predicted_altitude_error = math.sqrt(1 / ((1/fused_error_2) + (1/predicted_error_2)))
                self.changed = False
            if self.predicted_altitude is not None:
                if self.has_velocity_data:
                    current_time = rospy.Time()
                    delta_t = (current_time - self.last_update).to_sec()
                    self.last_update = current_time
                    self.predicted_altitude += delta_t * self.velocity
                    self.predicted_altitude_error = math.sqrt(math.pow(self.predicted_altitude_error, 2) + math.pow(delta_t, 2) * math.pow(self.velocity_error, 2))
                kalman_msg = AltitudeStamped()
                kalman_msg.value = self.predicted_altitude
                kalman_msg.error = self.predicted_altitude_error
                kalman_msg.stamp = rospy.Time()
                self.pub.publish(kalman_msg)
            # Sleep for the remainder of the loop
            self.lock.release()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('altitude_kalman_node')
    try:
        AltitudeKalman()
    except rospy.ROSInterruptException:
        pass