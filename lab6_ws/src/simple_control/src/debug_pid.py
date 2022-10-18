#!/usr/bin/env python
import math, sys, copy, random
from velocity_pid_class import PID
import matplotlib.pyplot as plt
import numpy as np
def parse_paramstring(paramstring):
    try:
        return eval(paramstring)
    except:
        print("Error parsing params; returning default [0.0, 0.0, 0.0]")
        return [0.0, 0.0, 0.0]

def init_PID(paramstring):
    params = parse_paramstring(paramstring)
    print("Initialized PID with params p={} i={} d={}".format(params[0], params[1], params[2]))
    return PID(params[0], params[1], params[2])

def add_noise(x):
    return x + random.gauss(0.0, 0.3)

def distance(p1, p2):
    return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))

def draw_timestep(poses, waypoints):
    plt.ion()
    plt.plot([i[0] for i in poses], [i[1] for i in poses], 'bo--', linewidth=2, markersize=12)
    plt.plot([waypoint[0] for waypoint in waypoints], [waypoint[1] for waypoint in waypoints], 'ro')
    plt.show()
    plt.pause(0.1)

# To invoke: python2 debug_pid.py [pxy, ixy, dxy]
# takes in a set of waypoints and PID parameters and simulates a PID
def main():
    waypoints = np.array([[5,5],[-5,5],[-5,-5],[5,-5],[5,5]])
    epsilon = 0.2
    pose = [0.0, 0.0]
    dt = 1
    x_pid = init_PID(sys.argv[1])
    y_pid = init_PID(sys.argv[1])
    poses = [copy.deepcopy(pose)]
    for i in range(len(waypoints)):
        print("New waypoint:{}".format(waypoints[i]))
        while distance(pose, waypoints[i]) > epsilon:
            x_err = waypoints[i][0] - pose[0]
            y_err = waypoints[i][1] - pose[1]
            pose[0] = add_noise(pose[0] + x_pid.pid_loop(x_err, dt))
            pose[1] = add_noise(pose[1] + y_pid.pid_loop(y_err, dt))
            poses.append(copy.deepcopy(pose))

            draw_timestep(poses, waypoints)

    print("Finished waypoints!")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
