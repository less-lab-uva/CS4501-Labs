#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float64

# Create the rocket engine class
class RocketEngine():
    # On node initilization
    def __init__(self):
        # Create the publisher and subscriber
        self.position_sub = rospy.Subscriber('/cmd_vel', Float64, self.velocityCallback, queue_size = 1)
        self.velocity_pub = rospy.Publisher('/sensor_vel', Float64, queue_size=1)
        

        # Create the sensor reading message we will send
        self.sensor_vel = Float64()
        self.sensor_vel.data = 0

        # Call the mainloop of our class
        self.mainloop()
        
    # Callback for the command velocity
    def velocityCallback(self, msg):
        # Set the sensor mean and deviation
        mu, sigma = 0, 10
        # Generate gaussian random noise
        s = np.random.normal(mu, sigma, 1)
        # Sensor reading is the velocity command + gaussian noise
        self.sensor_vel.data = msg.data + s[0]

    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(10)
    
        # While ROS is still running
        while not rospy.is_shutdown():
            
            # Publish the sensor reading
            self.velocity_pub.publish(self.sensor_vel)

            # Sleep for the remainder of the loop
            rate.sleep()

# Main function
if __name__ == '__main__':
    # Initilize the node
    rospy.init_node('RocketEngineNode')
    try:
        # Launch the rocket class
        rocket = RocketEngine()
    except rospy.ROSInterruptException:
        pass