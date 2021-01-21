#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Int64.h"

int main(int argc, char **argv)
{
    // Initialize the ROS node and register it with the roscore
    ros::init(argc, argv, "CountDownNode");
    ros::NodeHandle n;
    
    // Create a publisher to publish the velocity
    ros::Publisher countdown_pub = n.advertise<std_msgs::Int64>("countdown", 1000);

    // Create a variable which will control the countdown starting at 10
    int count = 10;

    //Wait for 2 seconds before we start the countdown
    sleep(5);

    // Set the rate of this node to 1Hz
    ros::Rate loop_rate(1);

    // While ROS is still running
    while (ros::ok())
    {
        // Create the ros message
        std_msgs::Int64 msg;
        msg.data = count;

        // If the count is greater than 0 subtract 1
        if (count >= 0)
        {
            // Publish the velocity
            countdown_pub.publish(msg);
            count--;
        }

        // Wait the amonut of time required to maintain 10Hz
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}