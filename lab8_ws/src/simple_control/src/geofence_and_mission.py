#!/usr/bin/env python
import rospy
import time
import copy
from enum import Enum
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from simple_control.srv import toggle_cage, toggle_cageResponse


# A class to keep track of the quadrotors state
class DroneState(Enum):
  HOVERING = 1
  VERIFYING = 2
  MOVING = 3


# Create a class which takes in a position and verifies it, keeps track of the state and publishes commands to a drone
class GeofenceAndMission():

  # Node initialization
  def __init__(self):

    # Create the publisher and subscriber
    self.position_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
    self.keyboard_sub = rospy.Subscriber('/uav/input/position_request', Vector3, self.getKeyboardCommand, queue_size = 1)

    self.service = rospy.Service('toggle_cage', toggle_cage, self.toggle_cageFunction)
    self.cage_on = True

    # TO BE COMPLETED FOR CHECKPOINT 2
    # TODO: Add a position_sub that subscribes to the drones pose
    self.keyboard_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.getPosition, queue_size = 1)

    # TO BE COMPLETED FOR CHECKPOINT 3
    # TODO: Get the cage parameters
    # Get the acceptance range
    self.acceptance_range = rospy.get_param("/geofence_and_mission/acceptance_range", 0.5)

    # Getting the virtual cage parameters
    cage_params = rospy.get_param('/geofence_and_mission/virtual_cage', {'x': 5, 'y': 5, 'z': 5})
    cx, cy, cz = cage_params['x'], cage_params['y'], cage_params['z']

    # Create the virtual cage

    self.cage_x = [-1 * cx, cx]
    self.cage_y = [-1 * cy, cy]
    self.cage_z = [0, cz]

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Launching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": Param: cage x - " + str(self.cage_x))
    rospy.loginfo(str(rospy.get_name()) + ": Param: cage y - " + str(self.cage_y))
    rospy.loginfo(str(rospy.get_name()) + ": Param: cage z - " + str(self.cage_z))
    rospy.loginfo(str(rospy.get_name()) + ": Param: acceptance range - " + str(self.acceptance_range))

    # Create the drones state as hovering
    self.state = DroneState.HOVERING
    rospy.loginfo(str(rospy.get_name()) + ": Current State: HOVERING")
    # Create the goal messages we are going to be sending
    self.goal_cmd = Vector3()
    self.unverified_goal_cmd = Vector3()

    # Create a point message that saves the drones current position
    self.drone_position = Point()

    # Start the drone a off the ground
    self.goal_cmd.z = 5.0

    # Keeps track of whether the position was changed or not
    self.goal_changed = False

    # Call the mainloop of our class
    self.mainloop()

  def toggle_cageFunction(self, request):
    self.cage_on = request.cage_on
    return toggle_cageResponse(True)

  # Callback for the keyboard manager
  def getKeyboardCommand(self, msg):
    # Save the keyboard command
    if self.state == DroneState.HOVERING:
      self.goal_changed = True
      self.unverified_goal_cmd = copy.deepcopy(msg)


  # TO BE COMPLETED FOR CHECKPOINT 2
  # TODO: Add function to receive drone's position messages
  def getPosition(self, msg):
    # Save the keyboard command
    self.drone_position = msg.pose.position

  # Converts a position to string for printing
  def goalToString(self, msg):
    pos_str = "(" + str(msg.x)
    pos_str += ", " + str(msg.y)
    pos_str += ", " + str(msg.z) + ")"
    return pos_str

  # TO COMPLETE FOR CHECKPOINT 4
  # TODO: Implement processVerifying
  def processVerifying(self):
    # Check if the new goal is inside the cage
    x_check = self.cage_x[0] <= self.unverified_goal_cmd.x <= self.cage_x[1]
    y_check = self.cage_y[0] <= self.unverified_goal_cmd.y <= self.cage_y[1]
    z_check = self.cage_z[0] <= self.unverified_goal_cmd.z <= self.cage_z[1]

    if not self.cage_on:
      rospy.loginfo(str(rospy.get_name()) + ": Current State: MOVING")
      self.state = DroneState.MOVING
      self.goal_cmd = self.unverified_goal_cmd
      return

    # If it is change state to moving
    if x_check and y_check and z_check:
      rospy.loginfo(str(rospy.get_name()) + ": Current State: MOVING")
      self.state = DroneState.MOVING
      self.goal_cmd = self.unverified_goal_cmd
    # If it is not change to hovering
    else:
      rospy.loginfo(str(rospy.get_name()) + ": Current State: HOVERING")
      self.state = DroneState.HOVERING

  # This function is called when we are in the hovering state
  def processHovering(self):
    # Print the requested goal if the position changed
    if self.goal_changed:
      rospy.loginfo(str(rospy.get_name()) + ": Requested Position: " + self.goalToString(self.goal_cmd))
      rospy.loginfo(str(rospy.get_name()) + ": Current State: VERIFYING")
      #  TO BE COMPLETED FOR CHECKPOINT 4
      # TODO: Update
      self.state = DroneState.VERIFYING
      self.goal_changed = False


  # This function is called when we are in the moving state
  def processMoving(self):
    # Compute the distance between requested position and current position
    dx = self.goal_cmd.x - self.drone_position.x
    dy = self.goal_cmd.y - self.drone_position.y
    dz = self.goal_cmd.z - self.drone_position.z

    print("Current Drone Position: \t(" + str(round(self.drone_position.x, 1)) + ", " + str(round(self.drone_position.y, 1)) + ", " + str(round(self.drone_position.z, 1)) + ")")
    print("Current Waypoint: \t\t(" + str(self.goal_cmd.x) + ", " + str(self.goal_cmd.y) + ", " + str(self.goal_cmd.z) + ")")
    print("-------------------")

    # Euclidean distance
    distance_to_goal = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
    # If goal is reached transition to hovering
    if distance_to_goal < self.acceptance_range:
      self.state = DroneState.HOVERING
      rospy.loginfo(str(rospy.get_name()) + ": Complete")
      rospy.loginfo(str(rospy.get_name()) + ": ----------------------------------")
      rospy.loginfo(str(rospy.get_name()) + ": Current State: HOVERING")


  # The main loop of the function
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(20)

    # While ROS is still running
    while not rospy.is_shutdown():
      # Publish the position
      self.position_pub.publish(self.goal_cmd)

      # Check if the drone is in a moving state
      if self.state == DroneState.MOVING:
        self.processMoving()
      # If we are hovering then accept keyboard commands
      elif self.state == DroneState.HOVERING:
        self.processHovering()
      # TODO: Add verifying state for CHECKPOINT 4
      elif self.state == DroneState.VERIFYING:
        self.processVerifying()

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('geofence_and_mission')
  try:
    ktp = GeofenceAndMission()
  except rospy.ROSInterruptException:
    pass
