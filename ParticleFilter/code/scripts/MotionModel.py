import sys
import numpy as np
import math

from MapReader import MapReader
from SensorModel import SensorModel
from Resampling import Resampling

from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """


    def __init__(self):

        """
        TODO : Initialize Motion Model parameters here
        """

        self.alpha1=1e-5
        self.alpha2=1e-5
        self.alpha3=6e-3
        self.alpha4=6e-3

        # self.alpha1=0
        # self.alpha2=0
        # self.alpha3=0
        # self.alpha4=0





    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        """
        TODO : Add your code here
        """

        del_rot1 = math.atan2(u_t1[1]-u_t0[1], u_t1[0]-u_t0[0]) - u_t0[2]
        del_trans = np.sqrt((u_t1[0]-u_t0[0])**2 + (u_t1[1]-u_t0[1])**2)
        del_rot2 = u_t1[2] -u_t0[2] - del_rot1

        std_rot1 = np.sqrt(self.alpha1*(del_rot1**2) + self.alpha2*(del_trans**2))
        std_trans  = np.sqrt(self.alpha3*(del_trans**2) + self.alpha4*(del_rot1**2) + self.alpha4*(del_rot2**2))
        std_rot2 = np.sqrt(self.alpha1*(del_rot2**2) + self.alpha2*(del_trans**2))

        #print("Scale",self.alpha3*(del_trans**2) + self.alpha4*(del_rot1**2) + self.alpha4*(del_rot2))
        del_rot1_sample = del_rot1 - np.random.normal(loc=0,scale=std_rot1)
        del_trans_sample = del_trans - np.random.normal(loc=0,scale=std_trans)
        del_rot2_sample = del_rot2 -np.random.normal(loc=0,scale=std_rot2)

        x1 = x_t0[0] + del_trans_sample*np.cos(x_t0[2] + del_rot1_sample)
        y1=  x_t0[1] + del_trans_sample*np.sin(x_t0[2] + del_rot1_sample)
        theta1 = x_t0[2] + del_rot1_sample + del_rot2_sample

        x_t1= np.array([x1,y1,theta1])

        #print(x_t1)

        return x_t1

if __name__=="__main__":
    #motion_model = MotionModel()

    """
        Code for testing motion model
    """
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    logfile = open(src_path_log, 'r')

    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map)
    resampler = Resampling()

    num_particles = 1
    X_bar = np.array([[5000, 4000, np.pi,1]])

    vis_flag = 1


    first_time_idx = True

    x_est_odom = []
    y_est_odom = []
    x_real_odom=[]
    y_real_odom=[]
    X_list = np.zeros((1,3))
    t_list = np.zeros(1)
    odom_robot_list = np.zeros((1,3))
    for time, line in enumerate(logfile):

        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        odometry_robot = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        time_stamp = meas_vals[-1]

        if (meas_type == "L"):
            odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
            ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan

        if (first_time_idx):
            u_t0 = odometry_robot
            first_time_idx = False
            continue
        x_real_odom.append(odometry_robot[0])
        y_real_odom.append(odometry_robot[1])
        X_bar_new = np.zeros( (num_particles,4), dtype=np.float64)
        u_t1 = odometry_robot

        for m in range(0, num_particles):
            x_t0 = X_bar[0,0:3]
            x_t1 = motion_model.update(u_t0, u_t1, x_t0)
            w_t=1
            X_bar_new[m,:] = np.hstack((x_t1, w_t))
        X_bar = X_bar_new
        u_t0 = u_t1

        x_est_odom.append(X_bar[0][0])
        y_est_odom.append(X_bar[0][1])


    fig = plt.figure()
    # plt.switch_backend('TkAgg')
    #mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    #plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);
    plt.plot(np.array(x_est_odom)/10.0, np.array(y_est_odom)/10.0,'o',color='red')
    plt.plot(np.array(x_real_odom)/10.0,np.array(y_real_odom)/10.0,'o',color='blue')

    plt.show()
    plt.pause(0)
