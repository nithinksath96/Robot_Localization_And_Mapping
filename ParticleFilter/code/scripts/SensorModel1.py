import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb

from MapReader import MapReader
from MotionModel import MotionModel

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
        self.laser_frame_offset = 25
        self.max_laser_range = 8100
        self.range_res = 2
        self.sample_degrees = 10.0

    def get_distance(self, present_laser_state, present_obstacle_state):

        # print("calculating distance ")
        # print("laser state is" + str(present_laser_state[0]) + "and" + str(present_laser_state[1]))
        # print("obstacle state is" + str(present_obstacle_state[0]) + "and" + str(present_obstacle_state[1]))

        distance_value = np.linalg.norm(present_obstacle_state - present_laser_state)

        # distance_vector = present_obstacle_state - present_laser_state



        # # Need to multiply by 10 for real distance
        # distance_vector = distance_vector

        # distance_value = np.dot(distance_vector.T, distance_vector)

        # distance_value = math.sqrt(distance_value)
        # print("end distance")

        # print("distance value is " + str(distance_value))

        # print("----------------------------------------")
        return distance_value

    def ray_cast_laser_state(self, X_laser_state):

        laser_range_list = []

        laser_angles = np.arange(-90.0,90.0,self.sample_degrees)
        laser_angles = (np.pi/180.0)*laser_angles
        laser_angles = laser_angles + X_laser_state[2]

        plot_x_list = []
        plot_y_list = []
        distance_plots = []

        X_laser_2D_state = np.transpose(np.array([X_laser_state[0], X_laser_state[1]]))

        for each_angle in laser_angles:

            forward_projection_vector = np.transpose(np.array([  self.range_res*math.cos(each_angle), self.range_res*math.sin(each_angle)]))
            # print(forward_projection_vector)
            obstacle_found = False
            present_range = np.array( np.transpose(np.array([X_laser_state[0], X_laser_state[1]])))
            # print(present_range)
            iter_count = 0

            while(True):

                present_range = present_range + forward_projection_vector

                x_index_present_range = int(( present_range[0]/10.0 ))
                y_index_present_range = int(( present_range[1]/10.0 ))


                if(x_index_present_range >= 800 or y_index_present_range >= 800):
                    # print("HI")
                    # print(self.max_laser_range)
                    # print(math.degrees(each_angle))
                    laser_range_list.append(self.max_laser_range)
                    break

                if(x_index_present_range < 0 or y_index_present_range < 0):
                    # print("HI")
                    # print(self.max_laser_range)
                    # print(math.degrees(each_angle))
                    laser_range_list.append(self.max_laser_range)
                    break

                if(self.occupancy_map[y_index_present_range, x_index_present_range] >= 0.1):
                    # print(self.occupancy_map[y_index_present_range, x_index_present_range])
                    obstacle_found = True
                    obstacle_state = np.transpose(np.array([x_index_present_range*10.0, y_index_present_range*10.0]))
                    # print(obstacle_state)
                    X_laser_2D_state_func = np.transpose(np.array([int(X_laser_2D_state[0]),int(X_laser_2D_state[1])]))
                    real_distance = self.get_distance(X_laser_2D_state_func, obstacle_state)
                    # print("real distance is " + str(real_distance))
                    laser_range_list.append(real_distance)

                    # if(real_distance > 20):
                    #     print(real_distance)
                    #     print("here")
                    # print("here2")

                    plot_x_list.append(X_laser_state[0]/10)
                    plot_x_list.append(x_index_present_range)

                    plot_y_list.append(X_laser_state[1]/10)
                    plot_y_list.append(y_index_present_range)
                    # distance_plots.append(real_distance)

                    break
                iter_count+=1
                # print("next run")

            # break

        # print(len(plot_x_list))
        # print(len(plot_y_list))



        # for each_distance in distance_plots:
        #     print(each_distance)

        # fig = plt.figure()
        # plt.imshow(self.occupancy_map, cmap='Greys')
        # plt.axis([0, 800, 0, 800])


        # for i in range(0, len(plot_x_list), 2):
        #     plt.plot(plot_x_list[i:i+2], plot_y_list[i:i+2], 'ro-')

        # plt.show()

        return laser_range_list

    def get_hit_prob(self, zk_star, zk):

        if(zk_star < 0 or zk_star > self.max_laser_range):
            return 0.0

        eta_hit_prob = 1.0

        hit_variance = 250.0

        term1 = 1./math.sqrt(2*np.pi*(hit_variance*hit_variance))
        term2 = np.exp(-0.5*(  np.square(zk - zk_star) / np.square(hit_variance)    )  )
        # hit_prob = eta_hit_prob*norm(zk_star,hit_variance).pdf(zk)
        hit_prob = term1 * term2

        return hit_prob

    def get_short_prob(self, zk_star, zk):

        if(zk >= 0 and zk <= zk_star):
            lambda_short = 0.003
            eta_short = 1/(1 - np.exp(-lambda_short*zk_star))

            short_prob = eta_short * lambda_short * np.exp(-lambda_short*zk)
            return short_prob
        else:
            return 0.0

    def get_max_prob(self, zk_star, zk):
        if(zk >= self.max_laser_range):
            return 1.0
        else:
            return 0.0

    def get_rand_prob(self, zk_star, zk):

        if(zk >= 0 and zk <= self.max_laser_range):
            return 1/(float(self.max_laser_range))
        else:
            return 0.0

    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """

        """
        TODO : Add your code here
        """
        print("hi")
        z_t1_arr = np.array(z_t1_arr)
        # print(z_t1_arr)
        z_t1_arr = z_t1_arr[::int(self.sample_degrees)]
        # print("hi inside beam_range_finder_model")
        x_robot = x_t1[0]
        y_robot = x_t1[1]
        theta_robot = x_t1[2]

        x_laser = x_robot +  self.laser_frame_offset*math.cos(theta_robot)
        y_laser = y_robot +  self.laser_frame_offset*math.sin(theta_robot)
        theta_laser = theta_robot

        X_laser = np.transpose(np.array([x_laser, y_laser, theta_laser]))

        # print(X_laser)


        laser_zk_stars = self.ray_cast_laser_state(X_laser)

        # print(len(laser_zk_stars))

        # for each_zk in laser_zk_stars:
        #     print(each_zk)


        #     print("----------------")

        # plt.scatter(x_robot, y_robot, c='r', marker='o')
        # plt.axis([0,800,0,800])
        # plt.figure()
        # plt.plot(laser_zk_stars)
        # plt.show()




        log_probability = 0.0
        # prob_return = []
        for index, angle in enumerate(z_t1_arr):

            present_laser_zk = z_t1_arr[index]
        #     # print(present_laser_zk)
            present_laser_zk_star = laser_zk_stars[index]

        #     # print(present_laser_range_belief)
        #     # print(present_laser_range_reading)
        #     # print("---------------------------")
            p_hit = self.get_hit_prob(present_laser_zk_star,present_laser_zk)
            p_short = self.get_short_prob(present_laser_zk_star,present_laser_zk)
            p_max = self.get_max_prob(present_laser_zk_star, present_laser_zk)
            p_rand = self.get_rand_prob(present_laser_zk_star, present_laser_zk)

            z_hit   = 1000
            z_short = 0.01
            z_max   = 0.03
            z_rand  = 100000

            # z_hit   = 2000.0
            # z_short = 0.01
            # z_max   = 0.03
            # z_rand  = 1e5

            overall_probability = z_hit*p_hit + z_short*p_short + z_max*p_max + z_rand*p_rand
        #     prob_return.append(overall_probability)
            if(overall_probability > 0):
                log_probability += np.log(overall_probability)

            # print("P_hit",z_hit*p_hit)
            # print("P_short",z_short*p_short)
            # print("P_max",z_max*p_max)
            # print("P_rand")

            # print(np.exp(log_probability))


        #     # print(p_hit)
        #     # print(p_short)
        #     # print(p_max)
        #     # print(p_rand)

        #     # p_max = self.get_max_prob()
        #     # p_rand = self.get_rand_prob()
        #     # break

        # # print(log_probability)
        # # print(np.exp(log_probability))
        # # print("-----------------------------------")
        # print(np.exp(log_probability))
        print("Total prob",np.exp(log_probability))
        return np.exp(log_probability)

    def plot_distribution(self):

        mean_vals = [500, 800, 1000]
        measure = range(0,self.max_laser_range+1)

        for mean_val in mean_vals:
            prob = []
            for mes in measure:
                p = 0.6480*self.get_hit_prob(mean_val, mes) + 0.025*self.get_short_prob(mean_val,mes) + 0.002*self.get_max_prob(mean_val,mes) + 0.325*self.get_rand_prob(mean_val, mes)
                prob.append(p)

            fig = plt.figure()
            plt.plot(measure, prob)
            plt.show()

def threshold_map(occ_map):

# for i in range(0, 800,1):
#     for j in range(0,800,1):
#         if occ_map[i,j] != 0.0:
#             occ_map[i,j] = 0.5

#         if occ_map[i,j] == 0.0:
#             occ_map[i,j] = 0.0

    occ_map[occ_map != 0.0] = 0.5
    occ_map[occ_map == 0.0] = 0.0

# print(occ_map[100,100])


# self._occupancy_map[self._occupancy_map != 0.0] == 0.5
# self._occupancy_map[self._occupancy_map == 0.0] == 0.0

# self._occupancy_map[self._occupancy_map == -1.0] == 1.0
    return occ_map

if __name__=='__main__':

    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    occupancy_map = threshold_map(occupancy_map)

    map_obj.visualize_map()

    sensor_model = SensorModel(occupancy_map)
    motion_model = MotionModel()

    num_particles = 1

    # X_bar = np.transpose(np.array([5.75300000e+02,  4.71200000e+02, -4.57011189e-01]))#This is obtained from random particles

    X_bar = np.transpose(np.array([4.4000000e+02,  3.95000000e+02, 3.14011189]))#This is obtained from random particles

    X_bar = np.transpose(np.array([4.4000000e+03,  3.95000000e+03, 0.00]))

    X_bar = X_bar.reshape((3,1))

    # test = X_bar[0:3,0]
    # rays  = sensor_model.ray_cast_laser_state(np.transpose(np.array([390,304,0])))
    # print(len(rays))
    logfile = open(src_path_log, 'r')
    first_time_idx = True
    found_l = False
    for time_idx, line in enumerate(logfile):
        meas_type = line[0]
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ')
        odometry_robot = meas_vals[0:3]
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


        for m in range(0, num_particles):

            x_t0 = X_bar[0:3,m]
            x_t1 = motion_model.update(u_t0, u_t1, x_t0)
            # print("x0 ----")
            # print(x_t0)
            # print("xt1----")
            # print(x_t1)
            # print("----")
            # print("----")
            probss = []
            if (meas_type == "L"):
                found_l = True
                z_t = ranges
                sensor_model.beam_range_finder_model(z_t, x_t1)
                X_bar_new = x_t1

                # w_t, prob_return = sensor_model.beam_range_finder_model(z_t, x_t1)
                # X_bar_new[m,:] = np.hstack((x_t1, w_t))
                # probss.append(w_t)
            else:
                X_bar_new = x_t1
                # print(X_bar)
                # print("----")
                # break

        X_bar = X_bar_new
        u_t0 = u_t1
        X_bar = X_bar.reshape((3,1))

        # if(found_l):
        #     break
        # else:
        #     print("continuing")







    #print(len(rays))

    # map_obj.visualize_map()

    # sensor_model.plot_distribution()
    # plt.show()
