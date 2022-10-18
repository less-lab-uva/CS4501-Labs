import os, shutil
import numpy as np
import math, copy, random
from threading import Lock

class WpGen:

    def __init__(self, points=None, velocity=0.2, hz=4):
        self.lock = Lock()
        self.hz = hz
        if points is None or len(points) == 0:
            self.points = self.continuous_sine_wave((20,20), 100)
        else:
            self.points = points
        print("self.points:")
        for i in range(len(self.points)):
            print("\t{}".format(self.points[i], prec=2))
        # if len(points) == 1:
        #     self.points = np.array([points[0],[0,0],points[0]])
        self.velocity = velocity


    # interpolate waypoints by distance of 0.1 (minimum velocity)
    def setup_trajectory_from_waypoints(self):
        if len(self.points) == 1:
            return self.points
        return self.interpolate_waypoints(self.points, self.velocity)


    # return points of circle-acting sine wave
    # centered in image
    def continuous_sine_wave(self, img_dims, num_points):
        points_arr = []
        R = 6 # radius
        a = 4 # amplitude
        n = 10 # number of peaks
        center = [0,0] #[img_dims[1]/2, img_dims[0]/2]
        prev_point = [0,0]
        prev_norm_point = [0,0]
        prev_angle = 0
        for theta in np.linspace(0,2 * math.pi,num_points):
            x = (R + a * math.sin(n*theta)) * math.cos(theta) + center[0]
            y = (R + a * math.sin(n*theta)) * math.sin(theta) + center[1]
            angle = math.atan2((x - prev_point[0]),(y - prev_point[1])) * 57.3
            #print("dx:{} dy:{} angle:{}".format((x - prev_point[0]), (y - prev_point[1]), angle))
            prev_point = copy.deepcopy([x, y])
            points_arr.append(copy.deepcopy([int(x), int(y), angle]))
            prev_angle = angle
        return points_arr


    # interpolate between waypoints with some velocity
    def interpolate_waypoints(self, waypoints, velocity=0.1):
        velocity = velocity / float(self.hz)
        interp_waypoints = []
        for i in range(len(waypoints[:-1])):
            point = waypoints[i]
            if i > 0:
                #print("last interp wayoint:"+str(interp_waypoints[-1][0:2]))
                point = interp_waypoints[-1][0:2]
                #print("point:"+str(point))
            distance = math.sqrt(math.pow(waypoints[i+1][0] - point[0], 2) + math.pow(waypoints[i+1][1] - point[1], 2))
            num_points = int(round(distance / velocity,0))
            dot = waypoints[i+1][0]*point[0] + waypoints[i+1][1]*point[1]
            det = point[0]*waypoints[i+1][1] - point[1]*waypoints[i+1][0]
            #angle = math.atan2(det, dot) + 3*math.pi /4
            angle = math.atan2((waypoints[i+1][1] - point[1]), (waypoints[i+1][0] - point[0]))
            #wp = np.append(point, math.degrees(angle))
            #interp_waypoints.append(wp)
            for d in range(num_points):
                x = point[0]+ (float(d)/num_points) * (waypoints[i+1][0] - point[0])
                y = point[1]+ (float(d)/num_points) * (waypoints[i+1][1] - point[1])
                interp_waypoints.append([x, y, math.degrees(angle)])
        return interp_waypoints


# runner to test class methods
def main(self):
    points = continuous_sine_wave(10, 100)
    # generate images
    dest_path = '/home/cs4501/Desktop/CS4501-Labs/lab6_ws/src/sensor_simulation/data/'
    with open(dest_path+"ship_viz/positions.txt", 'w') as f:
        for i, p in enumerate(points):
            rotated_overlay = rotate_image(overlay, p[2])      
            new_img = overlay_transparent(copy.deepcopy(background), rotated_overlay, p[0], p[1])
            new_img_filename = '{}ship_viz/ship{}.jpg'.format(dest_path, i)
            print("writing new image "+new_img_filename)
            cv2.imwrite(new_img_filename, new_img)
            p_env = translate_point_from_image_to_environment(p, background.shape)
            print("p_env:{}\n".format(p_env))
            # f.write("[{},{},0.0,0.0,0.0,{}]\n".format(p_env[0], p_env[1], p[2] / 57.3))


if __name__ == '__main__':
    main()