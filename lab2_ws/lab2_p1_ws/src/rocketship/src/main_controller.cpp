#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

// flag which sets when to start the mission
bool launch = false;
bool abort_mission = false;

// The function which is called everytime a countdown message is recieved
void countdownCallback(const std_msgs::Int64::ConstPtr& msg)
{
    // Cancel the countdown if the launch is aborted
    if (abort_mission == false)
    {
        ROS_INFO("Countdown: %lds", msg->data);
        // If the message is less or equal to 0
        if (msg->data <= 0)
        {
            // Launch the rocket
            launch = true;
        }
    }
}

// The function which is called everytime an abort message is recieved
void abortCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // If we get a true abort message
    if (msg->data == true)
    {
        // If we have not launched yet
        if (launch == false)
        {
            // Abort the mission
            ROS_INFO("Abort Abort Abort!!!");
            abort_mission = true;
        }
        else
        {
            // Otherwise we can not abort
            ROS_INFO("Abort Failed");
            abort_mission = false;
        }
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node and register it with the roscore
    ros::init(argc, argv, "MainControllerNode");
    ros::NodeHandle n;

    //Subscribe to the countdown and abort topic
    ros::Subscriber countdown_sub = n.subscribe("countdown", 1000, countdownCallback);
    ros::Subscriber abort_sub = n.subscribe("abort_takeoff", 1000, abortCallback);

    // Create a publisher to publish the velocity
    ros::Publisher velocity_pub = n.advertise<std_msgs::Float64>("cmd_vel", 1000);

    // Set the rate of this node to 10Hz
    ros::Rate loop_rate(10);

    // Launch the rocket
    ROS_INFO("Rocket Ready for Countdown");

    // Create the velocity variable
    double velocity = 0;

    // Create a variable which will control the velocity
    float vel_counter = -5;

    // While ROS is still running
    while (ros::ok())
    {
        // If we have launched and havent aborted
        if ((launch == true) && (abort_mission == false))
        {
            // Create the ros message
            std_msgs::Float64 msg;
            // We want the velocity to follow a Hyperbolic curve (1685m/s is the typical top speed of a falcon 9 rocket)
            velocity = (tanh(vel_counter) + 1) * 842;
            msg.data = velocity;

            // Every time the robot reachs an interval of 10mph display it to terminal
            ROS_INFO("Requested Velocity: %lfm/s", msg.data);

            // Publish the velocity
            velocity_pub.publish(msg);

            // incrreament the velocity counter
            vel_counter += 0.01 ;
        }

        // Wait the amonut of time required to maintain 10Hz
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
