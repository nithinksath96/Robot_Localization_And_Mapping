import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb

from MapReader import MapReader

import pylab as pl
from matplotlib import collections  as mc
import sys

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):

        """
        TODO : Initialize Sensor Model parameters here
        """
        self.occupancy_map = occupancy_map
        self.laser_displacement = 25.0 # In cm.
        self.laser_range = 8100 # In cm.
        self.laser_fov = [-89,90]  # In degrees
        self.laser_angular_step_degree = 10 # In degrees
        self.laser_radial_step = 2 # In centimeters
        self.laser_angular_step = self.laser_angular_step_degree * (np.pi / 180) #In radians
        self.map_resolution = 10.0 # In cm
        self.threshold = 0.8 #Threshold probability for detecting obstacles


        self.z_hit=1000.0
        self.z_short=0.01
        self.z_max=0.03
        self.z_rand=100000

        self.sigma_hit=250.0
        self.lambda_short=0.003

    def get_true_measurements(self, draw_flag):

        z_t1_star_x = []
        z_t1_star_y = []
        z_t1_star_dist = []
        z_t1_star_angle = []


        # Stepping through all directions
        for astep in range(self.laser_fov[0],self.laser_fov[1],self.laser_angular_step_degree):
            #print("Ray angle: ", astep)
            laser_x = self.laser_position[0]
            laser_y = self.laser_position[1]
            laser_theta = self.laser_position[2]
            current_ray_angle = astep
            current_global_ray_angle = laser_theta + np.radians(astep)
            ray_y = self.laser_radial_step * np.sin(current_global_ray_angle)
            ray_x = self.laser_radial_step * np.cos(current_global_ray_angle)

            for rstep in range(0, self.laser_range, self.laser_radial_step):

                laser_x += ray_x
                laser_y += ray_y


                map_x = int((laser_x)/self.map_resolution)
                map_y = int((laser_y)/self.map_resolution)

                if(map_x >= self.occupancy_map.shape[0] or map_y >= self.occupancy_map.shape[1] or map_x<0 or map_y<0):
                    break
                    # laser_x -= ray_x
                    # laser_y -= ray_y
                    # map_x = int((laser_x)/self.map_resolution)
                    # map_y = int((laser_y)/self.map_resolution)
                    # #print("Dont Know what to do yet")
                else:

                    # if(laser_y <=2350 and laser_y >= 2250 and laser_x >= 5150 and laser_x <= 5250):
                    #     print("Point: ", [laser_x,laser_y], "Angle: ", astep," Probs: ",self.occupancy_map[map_x,map_y], "Map coordinate: ",[map_x, map_y])
                    #     print("Occupancy map: ", self.occupancy_map[515:525,225:235])

                    if(self.occupancy_map[map_y, map_x] >= self.threshold):
                        #print("Obstacle found")
                        obstacle_point = np.array(laser_x, laser_y)
                        obst_dist = np.linalg.norm(self.laser_position[0:2]-obstacle_point)

                        z_t1_star_x.append(map_x)
                        z_t1_star_y.append(map_y)
                        z_t1_star_dist.append(obst_dist)
                        z_t1_star_angle.append(int(current_ray_angle + 90))

                        break

        if(draw_flag):
            # print("figure should be seen")
            # print("X: ", np.array(z_t1_star_x).T)
            # print("Y: ", np.array(z_t1_star_y).T)
            # print("Robot Position: ",)

            fig = plt.figure()
            # plt.switch_backend('TkAgg')

            mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())

            plt.ion(); plt.imshow(self.occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);

            plt.plot(z_t1_star_x, z_t1_star_y, 'o', color='red')

            plt.plot(self.laser_position[0]/self.map_resolution,self.laser_position[1]/self.map_resolution,'o',color='blue')
            plt.plot(self.bot_centroid[0]/self.map_resolution,self.bot_centroid[1]/self.map_resolution,'o',color='yellow')
            plt.pause(1)
            #plt.show()
            plt.close()
            #sys.exit()


        return z_t1_star_dist , z_t1_star_angle


    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        draw_flag = False

        # Location of the bot
        x = x_t1[0]
        y = x_t1[1]
        theta = x_t1[2]

        #Location of th laser
        print("In sensor model")
        self.laser_position = np.array([x+self.laser_displacement*np.cos(theta), y+self.laser_displacement*np.sin(theta), theta])
        self.bot_centroid = x_t1
        z_t1_star_dist, z_t1_star_angle = self.get_true_measurements(draw_flag)

        #input()
        #z_t1_star_dist=[0]*len(z_t1_arr)

        log_q=0

        for i in range(len(z_t1_star_dist)):

            ray_angle = z_t1_star_angle[i]-1
            #print(ray_angle)
            #norm_hit = 1.0/(0.5*(math.erf((self.laser_range -z_t1_star_dist[i])/math.sqrt(2)*self.sigma_hit) - (math.erf((-z_t1_star_dist[i])/math.sqrt(2)*self.sigma_hit))))
            p_hit = self.z_hit*np.random.normal(loc=z_t1_star_dist[i],scale=self.sigma_hit)
            if(z_t1_arr[ray_angle]<0 and z_t1_arr[ray_angle]>self.laser_range): p_hit=0.0

            norm_short = 1.0/(1.0-np.exp(-self.lambda_short*z_t1_star_dist[i]))
            p_short = self.z_short*norm_short*np.random.exponential(scale=1.0/self.lambda_short)
            if(z_t1_arr[ray_angle]<0 and z_t1_arr[ray_angle]>self.laser_range): p_short=0.0

            p_max=0.0
            if(z_t1_arr[ray_angle]==self.laser_range): p_max=self.z_max

            p_rand=0.0
            if(z_t1_arr[ray_angle]>=0 and z_t1_arr[ray_angle]<=self.laser_range): p_rand = self.z_rand*(1.0/self.laser_range)

            p_total = p_hit + p_short + p_max + p_rand

            log_q+=np.log(p_total)

            print("end")
            #break
        #print(log_q)
        return log_q














        """
        TODO : Add your code here
        """

        #return q

if __name__=='__main__':
    pass
