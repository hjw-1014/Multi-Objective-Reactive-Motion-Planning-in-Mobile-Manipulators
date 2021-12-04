import numpy as np
import pybullet as p
import time
import matplotlib.pyplot as plt

from cep.envs import Tiago_LeftParallelHand_Base
from cep.cep_models import cep_model_lefthandBase_taskgotoAndPathplan
import torch
from mat_animation import Plotting

joint_limit_buffers = 0.02
joint_limits = np.array([2.5, 2.5, 3.1416, 2.75, 1.57, 3.53, 2.35, 2.09, 1.57, 2.09]) - joint_limit_buffers

device = torch.device('cpu')


def plot_xyzd(des_x=1.7, des_y=1.1, des_z=0.8,
              x_mean=None, y_mean=None, z_mean=None, d_mean=None,
              x_std=None, y_std=None, z_std=None, d_std=None):

    fig, ax = plt.subplots(1, 4, figsize=(9, 9))
    print(type(x_mean))
    print(x_mean.shape[0])
    print(x_mean)
    l = x_mean.shape[0]
    print(l)
    x = np.linspace(1, l, l)
    print(x)
    print("====++=====")
    print(x_mean.shape)
    print(x_std.shape)
    print(x_mean+x_std)

    ax[0].fill_between(x, (x_mean-x_std), (x_mean+x_std), color='C0', alpha=0.2)
    ax[0].set_title("Position X")
    ax[0].plot(x, x_mean)
    ax[0].plot(x, np.ones(l) * des_x, linestyle=":")
    # ax[0].legend("Distance /m")

    ax[1].fill_between(x, (y_mean - y_std), (y_mean + y_std), color='C1', alpha=0.2)
    ax[1].set_title("Position Y")
    ax[1].plot(x, y_mean)
    ax[1].plot(x, np.ones(l) * des_y,  linestyle=":")

    ax[2].fill_between(x, (z_mean-z_std), (z_mean+z_std), color='C2', alpha=0.2)
    ax[2].set_title("Position Z")
    ax[2].plot(x, z_mean)
    ax[2].plot(x, np.ones(l) * des_z, linestyle=":")

    ax[3].fill_between(x, (d_mean-d_std), (d_mean+d_std), color='C3',alpha=0.2)
    ax[3].plot(x, d_mean)
    ax[3].set_title("Distance from goal")
    ax[3].plot(x, np.zeros(l),  linestyle=":")

    plt.show()

class CEPPolicy():
    def __init__(self, dt=1 / 10., dtype='float64'):
        self.dt = dt
        self.dtype = dtype

        self.controller = cep_model_lefthandBase_taskgotoAndPathplan()

    def policy(self, state):
        joint_poses = state[0, 0:10]
        joint_vels = state[0, 10:]

        action = self.controller.policy(state)

        x, dx = self.step(joint_poses, joint_vels, action)
        return x, dx


    def step(self, joint_poses, joint_vels, joint_accs):
        joint_vels = joint_vels + joint_accs * self.dt
        joint_poses = joint_poses + joint_vels * self.dt
        return joint_poses, joint_vels


def experiment():
    '''
    envlist:[it can take values [1,2,3]. It provides the type of environment we are using]
    results_dir: path to the folder in which we are saving the results
    '''

    time_step = 1 / 240.

    env = Tiago_LeftParallelHand_Base(time_step=time_step, Target_pose=[1.7, 1.1, 0.8])

    policy = CEPPolicy(dt=time_step)
    ################

    n_trials = 100
    horizon = 5000
    c = 0
    s = 0
    REWARD = 0
    END_POSITION = None

    X_List = []
    Y_List = []
    Z_List = []
    Dist_List = []

    for itr in range(n_trials):
        state = env.reset()
        p.addUserDebugLine([0., 0., -0.189], [1.5, 0., -0.189], [1., 0., 0.])

        robot_x_list = []
        robot_y_list = []
        dist_list = []
        ee_x_list = []
        ee_y_list = []
        ee_z_list = []
        ee_dist_list = []

        for i in range(horizon):
            init = time.time()
            print('###Iteration: ', i)
            #### Get Control Action (Position Control)####
            a = policy.policy(state)
            state, base_dist, done, success, q_vals = env.step(a)
            #print(state)

            ###################################
            # TODO: Record robot x and y values | 09.16
            robot_x_list.append(state[0][0])
            robot_y_list.append(state[0][1])
            dist_list.append(base_dist)

            REWARD = base_dist
            END_POSITION = env.check_endPosition()
            ee_dist = env._compute_reward()
            ee_x_list.append(END_POSITION[0])
            #print("END_POSITION[0]: ", END_POSITION[0])
            ee_y_list.append(END_POSITION[1])
            ee_z_list.append(END_POSITION[2])
            ee_dist_list.append(ee_dist)

            ###################################

            end = time.time()
            time.sleep(np.clip(time_step - (end - init), 0, time_step))

            # if i == (horizon-1) or ee_dist <= 0.02:
            #     # REWARD = base_dist
            #     # END_POSITION = env.check_endPosition()
            #     # ee_dist = env._compute_reward()
            #     # ee_x_list.append(END_POSITION[0])
            #     # print("END_POSITION[0]: ", END_POSITION[0])
            #     # ee_y_list.append(END_POSITION[1])
            #     # ee_z_list.append(END_POSITION[2])
            #     # ee_dist_list.append(ee_dist)
            #
            #     print('Position state: ', state[0])
            #     print('Distance:',  REWARD)
            #     print('End position: ', END_POSITION)
            #     print('Desired position', env.Target_pos)
            #
            #     ##################################
            #     # TODO: Matplot animation version | 09.16
            #     # print("len(robot_x_list): ", len(robot_x_list))
            #     # print("robot_x_list: ", robot_x_list)
            #     # print("robot_y_list: ", robot_y_list)
            #
            #     #plotting = Plotting(robot_x_list=ee_x_list, robot_y_list=ee_y_list, robot_z_list = ee_z_list, dist_list = ee_dist_list, horizon=i)
            #     #plotting.plot_fig_wholebody(des_x=1.7, des_y=1.1, des_z=0.8)
            #     #plotting.plot_path(robot_x_list=robot_x_list, robot_y_list=robot_y_list)
            #     #plotting.plot_animation()
            #     break
        ##################################
        X_List.append(np.asarray(ee_x_list).reshape(1, -1))
        Y_List.append(np.asarray(ee_y_list).reshape(1, -1))
        Z_List.append(np.asarray(ee_z_list).reshape(1, -1))
        Dist_List.append(np.asarray(ee_dist_list).reshape(1, -1))

        #plot_joints(q_list, horizon)

    X_list = np.concatenate(X_List, axis=0)
    Y_list = np.concatenate(Y_List, axis=0)
    Z_list = np.concatenate(Z_List, axis=0)
    Dist_list = np.concatenate(Dist_List, axis=0)

    X_mean = np.mean(X_list, axis=0)
    Y_mean = np.mean(Y_list, axis=0)
    Z_mean = np.mean(Z_list, axis=0)
    D_mean = np.mean(Dist_list, axis=0)

    X_std = np.std(X_list, axis=0)
    Y_std = np.std(Y_list, axis=0)
    Z_std = np.std(Z_list, axis=0)
    D_std = np.std(Dist_list, axis=0)

    plot_xyzd(x_mean=X_mean, y_mean=Y_mean, z_mean=Z_mean, x_std=X_std, y_std=Y_std, z_std=Z_std, d_mean=D_mean, d_std=D_std)

    p.disconnect()

# def experiment():
#     '''
#     envlist:[it can take values [1,2,3]. It provides the type of environment we are using]
#     results_dir: path to the folder in which we are saving the results
#     '''
#
#     time_step = 1 / 250
#
#     env = Tiago_LeftParallelHand_Base(time_step=time_step)
#
#     policy = CEPPolicy(dt=time_step)
#     ################
#
#     n_trials = 5
#     horizon = 5000
#     c = 0
#     s = 0
#     REWARD = 0
#     r = 10
#     END_POSITION = None
#     for itr in range(n_trials):
#         print('###Iteration: {}'.format(itr))
#         state = env.reset()
#         p.addUserDebugLine([0., 0., -0.189], [1.5, 0., -0.189], [1., 0., 0.])
#
#         q_list = []
#         for i in range(horizon):
#             init = time.time()
#
#             #### Get Control Action (Position Control)####
#             a = policy.policy(state)
#             state, reward, done, success, q_vals = env.step(a)
#             # TODO: Record joint values 07.10
#             q_list.append(q_vals)
#             #############################
#
#             end = time.time()
#             time.sleep(np.clip(time_step - (end - init), 0, time_step))
#
#             if i == (horizon-1):
#                 REWARD = reward
#                 END_POSITION = env.check_endPosition()
#         print('Position state: ', state[0])
#         print('Distance:',  REWARD)
#         print('End position: ', END_POSITION)
#         print('Desired position', env.Target_pos)
#         #plot_joints(q_list, horizon)
#
#     p.disconnect()


if __name__ == '__main__':
    p.connect(p.GUI_SERVER, 1234,
              options='--background_color_red=1. --background_color_green=1. --background_color_blue=1.')
    p.resetDebugVisualizerCamera(2.2, 55.6, -47.4, [0.04, 0.06, 0.31])
    experiment()

