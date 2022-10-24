# Quadrotor Simulator

This simulator is used in UVA's **Robotics for Software Engineers** course. 

## Prerequisites

This simulator has been tested on an Ubuntu machine with ROS installed. We have tested:

* Ubuntu 16.04 - [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Ubuntu 18.04 - [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

You will also need to install `catkin_tools`. The installation for this package can be found [here](https://catkin-tools.readthedocs.io/en/latest/installing.html).

## Installing the Simulator

First, we will start by cloning the repo:
```
$ cd ~
$ git clone https://github.com/hildebrandt-carl/fastersim.git
$ cd ~/fastersim
```

Next, install all the prerequisites required for this project:
```
$ sudo apt-get update
$ sudo apt-get install libsdl-dev python-pip -y
$ python2 -m pip install --upgrade pip
$ python2 -m pip install scipy
```

Now we are going to install the ROS dependent dependencies (**Note:** Change `kineitc` to `melodic` if you are using ROS melodic):
```
$ cd ~/fastersim
$ rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
$ sudo apt-get install ros-kinetic-joy ros-kinetic-imu-filter-madgwick -y
```

Next, we are going to compile to the workspace.
```
$ cd ~/fastersim
$ catkin build
```

## Using the Simulator

Finally, we can run the simulator
```
$ source ~/fastersim/devel/setup.bash
$ roslaunch flightcontroller fly.launch
```

If you want to test if its working open another terminal and run:
```
$ source ~/fastersim/devel/setup.bash
$ rosrun rqt_publisher rqt_publisher
```

Inside the rqt_publisher publish messages to:
**Topic:** /uav/input/position 
**Message Type:** geometry_msgs/Vector3
**Frequency:** 1Hz


Set the x,y, and z: the quadrotor will fly to that point.

## Testing the Simulator

To test the simulator, let it run for a minute. Do not provide the simulator with any input x,y, or z position. The default position is (0,0,3). The simulator should look as shown below. If the quadrotor (blue point) falls off the screen, try improving your VM's performance using the FAQ on the website. If you are sure you have tried all VM improvements, and the quadrotor still falls off the screen, let us know.

![](./sim_running.gif)

## Thanks

The flight dynamics are are adapted from [multicopterDynamicSim](https://github.com/mit-fast/multicopterDynamicsSim), while the GUI is adapted from [Quadcopter_simulator][https://github.com/abhijitmajumdar/Quadcopter_simulator]. A tip of the hat to both developers.