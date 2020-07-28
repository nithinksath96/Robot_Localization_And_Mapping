import numpy as np
import sys
import pdb

from MapReader import MapReader
from MotionModel import MotionModel
from SensorModel import SensorModel
from Resampling import Resampling

from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time

def visualize_map(occupancy_map):
    fig = plt.figure()
    # plt.switch_backend('TkAgg')
    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);


def visualize_timestep(X_bar, tstep):
    x_locs = X_bar[:,0]/10.0
    y_locs = X_bar[:,1]/10.0
    scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    plt.pause(0.00001)
    scat.remove()

def init_particles_random(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    y0_vals = np.random.uniform( 0, 7000, (num_particles, 1) )
    x0_vals = np.random.uniform( 3000, 7000, (num_particles, 1) )
    theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )

    # initialize weights for all particles
    w0_vals = np.ones( (num_particles,1), dtype=np.float64)
    w0_vals = w0_vals / num_particles

    X_bar_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))

    return X_bar_init

def init_particles_freespace(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles

    """
    TODO : Add your code here
    """

    return X_bar_init

def main():

    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    logfile = open(src_path_log, 'r')

    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map)
    resampler = Resampling()

    num_particles = 1
    #X_bar = np.transpose(np.array([5.75300000e+03,  1.71200000e+03, -4.57011189e-01]))
    X_bar = init_particles_random(num_particles, occupancy_map)
    print(X_bar.shape)
    vis_flag = 1


    first_time_idx = True

    x_est_odom = []
    y_est_odom = []


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

        X_bar_new = np.zeros( (num_particles,4), dtype=np.float64)
        u_t1 = odometry_robot
        flag=0
        for m in range(0, num_particles):
            #print("First",X_bar.shape)
            x_t0 = X_bar[0][0:3]
            #print(x_t0.shape)
            x_t1 = motion_model.update(u_t0, u_t1, x_t0)
            print("1---------",x_t1)
            #input()
            if (meas_type == "L"):
                z_t = ranges
                #w_t=1
                w_t = sensor_model.beam_range_finder_model(z_t, x_t1)
                # flag=1;
                # break
                # w_t = 1/num_particles
                print("2-----------------",X_bar_new)
                X_bar_new[m,:] = np.hstack((x_t1, w_t))
            else:
                X_bar_new[m,:] = np.hstack((x_t1, X_bar[m,3]))

            print("3-------------------",X_bar_new)
            X_bar_new[m,:] = np.hstack((x_t1, X_bar[m,3]))
            #print("Second",x_t1.shape)
            #print("Threee",X_bar_new.shape)
        #if(flag==1):
            #break
        print("4-------------------------",X_bar_new)
        X_bar = X_bar_new
        print("5--------------------------",X_bar)
        u_t0 = u_t1

        x_est_odom.append(X_bar[0][0])
        y_est_odom.append(X_bar[0][1])




    plt.imshow(occupancy_map, cmap='Greys')
    #plt.show()
    #plt.subplot(1,2,1)
    plt.draw()
    plt.subplot(1,2,1)
    plt.plot(x_est_odom, y_est_odom)
    plt.show()




if __name__=="__main__":
    main()
