import sys
import math
import copy

import numpy as np

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class GUI():
    # 'quad_list' is a dictionary of format: quad_list = {'quad_1_name':{'position':quad_1_position,'orientation':quad_1_orientation,'arm_span':quad_1_arm_span}, ...}
    def __init__(self, quads, env, map_size):
        # Used for the drawing
        self.quads = quads
        self.world = env
        self.fig = plt.figure()
        self.ax = Axes3D.Axes3D(self.fig)
        x = map_size[0]
        y = map_size[1]
        self.ax.set_xlim3d([-1 * x, x])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-1 * y, y])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([0, 15.0])
        self.ax.set_zlabel('Z')
        self.ax.set_title('Quadcopter Simulation')
        # Used to check if redrawing is required
        self.last_draw_path = np.zeros(10)
        self.last_draw_height = -1
        self.init_plot()

    def rotation_matrix(self,angles):
        ct = math.cos(angles[0])
        cp = math.cos(angles[1])
        cg = math.cos(angles[2])
        st = math.sin(angles[0])
        sp = math.sin(angles[1])
        sg = math.sin(angles[2])
        R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
        R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R

    def add_quad(self, key, props):
        if not key in self.quads:
            self.quads[key] = props
        self.quads[key]['l1'], = self.ax.plot([], [], [], color='blue', linewidth=3, antialiased=False)
        self.quads[key]['l2'], = self.ax.plot([], [], [], color='red', linewidth=3, antialiased=False)
        self.quads[key]['hub'], = self.ax.plot([], [], [], marker='o', color=self.quads[key]['color'], markersize=self.quads[key]['markersize'], antialiased=False)


    def init_plot(self):
        # Draw the quad
        for key in self.quads:
            self.quads[key]['l1'], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
            self.quads[key]['l2'], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
            self.quads[key]['hub'], = self.ax.plot([],[],[],marker='o',color='green', markersize=6,antialiased=False)
        # Draw the path
        path = self.world["path"]
        self.line, = self.ax.plot(path[:,0], path[:,1], np.full(len(path[:,1]), 5), marker=".", linestyle="--", markersize=10, color='C7')

    def draw_quad(self):
        for key in self.quads:
            R = self.rotation_matrix(self.quads[key]['orientation'])
            L = self.quads[key]['L']
            points = np.array([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T
            points = np.dot(R,points)
            points[0,:] += self.quads[key]['position'][0]
            points[1,:] += self.quads[key]['position'][1]
            points[2,:] += self.quads[key]['position'][2]
            self.quads[key]['l1'].set_data(points[0,0:2],points[1,0:2])
            self.quads[key]['l1'].set_3d_properties(points[2,0:2])
            self.quads[key]['l2'].set_data(points[0,2:4],points[1,2:4])
            self.quads[key]['l2'].set_3d_properties(points[2,2:4])
            self.quads[key]['hub'].set_data(points[0,5],points[1,5])
            self.quads[key]['hub'].set_3d_properties(points[2,5])

    def draw_path(self, path):
        self.line.set_xdata(path[:,0])
        self.line.set_ydata(path[:,1])
        self.line.set_3d_properties(np.full(len(path[:,1]), 5))
        self.last_draw_path = copy.deepcopy(path)

    def draw_world(self, height):
        self.ax.collections = []
        for obs in self.world["environment"]:
            obs_x = obs[0]
            obs_y = obs[1]
            x_obs = [obs_x, obs_x + 1, obs_x + 1, obs_x]
            y_obs = [obs_y, obs_y, obs_y + 1, obs_y + 1]
            z_obs = [height, height, height, height]
            verts = [list(zip(x_obs, y_obs, z_obs))]
            self.ax.add_collection3d(Poly3DCollection(verts))
            self.last_draw_height = height

    def update(self):
        # Draw the quad
        self.draw_quad()

        # Draw the path if it hasnt updated
        path = self.world['path']
        if not np.array_equal(path, self.last_draw_path):
            self.draw_path(path)

        # Draw the obstacles
        current_height = self.quads[self.quads.keys()[0]]['position'][2]
        if abs(current_height - self.last_draw_height) > 0.5:
            self.draw_world(current_height)

        # Update the plot
        plt.pause(0.000000000000001)
