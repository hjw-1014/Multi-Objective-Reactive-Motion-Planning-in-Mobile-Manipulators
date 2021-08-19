import numpy as np
import pybullet as p
import time
import matplotlib.pyplot as plt

from cep.envs import Tiago_LeftParallelHand_Base
from cep.cep_models import cep_simple_model_tiago_lefthand_base
import torch

joint_limit_buffers = 0.02
joint_limits = np.array([2.5, 2.5, 3.1416, 2.75, 1.57, 3.53, 2.35, 2.09, 1.57, 2.09]) - joint_limit_buffers

device = torch.device('cuda')


class CEPPolicy():
    def __init__(self, dt=1 / 240., dtype='float64'):
        self.dt = dt
        self.dtype = dtype

        self.controller = cep_simple_model_tiago_lefthand_base()

    def policy(self, state):
        joint_poses = state[0, 0:10]
        joint_vels = state[0, 10:]

        action = self.controller.policy(state)

        x, dx = self.step(joint_poses, joint_vels, action)
        return x, dx


    def step(self, joint_poses, joint_vels, joint_accs):
        joint_poses = joint_poses + joint_vels * self.dt
        joint_vels = joint_vels + joint_accs * self.dt
        return joint_poses, joint_vels

def plot_joints(joint_values: list, num: int):

    fig, axs = plt.subplots(2, 4)
    t = np.arange(0, num, 1)

    j1 = []
    j2 = []
    j3 = []
    j4 = []
    j5 = []
    j6 = []
    j7 = []

    for i in range(num):
        j1.append(joint_values[i][0])
        j2.append(joint_values[i][1])
        j3.append(joint_values[i][2])
        j4.append(joint_values[i][3])
        j5.append(joint_values[i][4])
        j6.append(joint_values[i][5])
        j7.append(joint_values[i][6])

    axs[0, 0].plot(t, j1)
    axs[0, 0].set_title('1st Joint')
    axs[0, 0].set_ylim(min(j1) - 1, max(j1) + 1)

    axs[0, 1].plot(t, j2)
    axs[0, 1].set_title('2nd Joint')
    axs[0, 1].set_ylim(min(j2) - 1, max(j2) + 1)

    axs[0, 2].plot(t, j3)
    axs[0, 2].set_title('3rd Joint')
    axs[0, 2].set_ylim(min(j3) - 1, max(j3) + 1)

    axs[0, 3].plot(t, j4)
    axs[0, 3].set_title('4th Joint')
    axs[0, 3].set_ylim(min(j4) - 1, max(j4) + 1)

    axs[1, 0].plot(t, j5)
    axs[1, 0].set_title('5th Joint')
    axs[1, 0].set_ylim(min(j5) - 1, max(j5) + 1)

    axs[1, 1].plot(t, j6)
    axs[1, 1].set_title('6th Joint')
    axs[1, 1].set_ylim(min(j6) - 1, max(j6) + 1)

    axs[1, 2].plot(t, j7)
    axs[1, 2].set_title('7th Joint')
    axs[1, 2].set_ylim(min(j7) - 1, max(j7) + 1)

    plt.show()

def experiment():
    '''
    envlist:[it can take values [1,2,3]. It provides the type of environment we are using]
    results_dir: path to the folder in which we are saving the results
    '''

    time_step = 1 / 250.

    env = Tiago_LeftParallelHand_Base(time_step=time_step)

    policy = CEPPolicy(dt=time_step)
    ################

    n_trials = 100
    horizon = 1500
    c = 0
    s = 0
    REWARD = 0
    END_POSITION = None
    for itr in range(n_trials):
        print('###Iteration: {}'.format(itr))
        state = env.reset()
        p.addUserDebugLine([0., 0., -0.189], [1.5, 0., -0.189], [1., 0., 0.])

        q_list = []
        for i in range(horizon):
            init = time.time()

            #### Get Control Action (Position Control)####
            a = policy.policy(state)
            state, reward, done, success, q_vals = env.step(a)
            #print(state)
            # TODO: Record joint values 07.10
            #q_list.append(q_vals)
            #############################

            end = time.time()
            time.sleep(np.clip(time_step - (end - init), 0, time_step))

            if i == (horizon-1):
                REWARD = reward
                END_POSITION = env.check_endPosition()
        print('Position state: ', state[0])
        print('Distance:',  REWARD)
        print('End position: ', END_POSITION)
        print('Desired position', env.Target_pos)
        #plot_joints(q_list, horizon)
    p.disconnect()


if __name__ == '__main__':
    p.connect(p.GUI_SERVER, 1234,
              options='--background_color_red=1. --background_color_green=1. --background_color_blue=1.')
    p.resetDebugVisualizerCamera(2.2, 55.6, -47.4, [0.04, 0.06, 0.31])
    experiment()

