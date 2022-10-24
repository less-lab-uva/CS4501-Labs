import sys

import numpy as np
import math
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import matplotlib.animation as animation


def gui_shutdown_hook():
    plt.close()

def rotation_matrix(angles):
    ct = math.cos(angles[0])
    cp = math.cos(angles[1])
    cg = math.cos(angles[2])
    st = math.sin(angles[0])
    sp = math.sin(angles[1])
    sg = math.sin(angles[2])
    R_x = np.array([[1, 0,    0],
                    [0, ct, -st],
                    [0, st,  ct]])
    R_y = np.array([[cp,  0, sp],
                    [0,   1,  0],
                    [-sp, 0, cp]])
    R_z = np.array([[cg, -sg, 0],
                    [sg,  cg, 0],
                    [0,    0,  1]])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R


class GUI:
    # 'quad_list' is a dictionary of format: quad_list = {'quad_1_name':{'position':quad_1_position,'orientation':quad_1_orientation,'arm_span':quad_1_arm_span}, ...}
    def __init__(self, quads, rate, yield_func):
        self.quads = quads
        self.rate = rate
        self.interval = int(1000 / self.rate)
        self.fig = plt.figure()
        self.ax = Axes3D.Axes3D(self.fig)
        self.ax.set_xlim3d([-10.0, 10.0])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-10.0, 10.0])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([0, 15.0])
        self.ax.set_zlabel('Z')
        self.ax.set_title('Quadcopter Simulation')
        self.init_plot()
        self.ani = animation.FuncAnimation(self.fig, self.update, yield_func, interval=self.interval, blit=True)
        plt.show(block=True)

    def init_plot(self):
        for key in self.quads:
            self.quads[key]['l1'], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
            self.quads[key]['l2'], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
            self.quads[key]['hub'], = self.ax.plot([],[],[],marker='o',color='green', markersize=6,antialiased=False)

    def update(self, updated_quads):
        redrawn = []
        for key, quad in self.quads.items():
            R = rotation_matrix(updated_quads[key]['orientation'])
            L = quad['L']
            points = np.array([[-L, 0, 0], [L, 0, 0], [0, -L, 0], [0, L, 0], [0, 0, 0], [0, 0, 0]]).T
            points = np.dot(R, points)
            points[0, :] += updated_quads[key]['position'][0]
            points[1, :] += updated_quads[key]['position'][1]
            points[2, :] += updated_quads[key]['position'][2]
            quad['l1'].set_data(points[0, 0:2], points[1, 0:2])
            quad['l1'].set_3d_properties(points[2, 0:2])
            quad['l2'].set_data(points[0, 2:4], points[1, 2:4])
            quad['l2'].set_3d_properties(points[2, 2:4])
            quad['hub'].set_data(points[0, 5], points[1, 5])
            quad['hub'].set_3d_properties(points[2, 5])
            redrawn.append(quad['l1'])
            redrawn.append(quad['l2'])
            redrawn.append(quad['hub'])
        return redrawn
