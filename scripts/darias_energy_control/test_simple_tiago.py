import numpy as np
import pybullet as p
import time


from cep.envs import TiagoOneParallelHand
from cep.cep_models import cep_simple_model_tiago

# from cep.envs import DariasHandSimple
# from cep.cep_models import cep_simple_model
import torch


joint_limit_buffers = 0.02
joint_limits = np.array([2.75, 1.57, 3.53, 2.35, 2.09, 1.57, 2.09]) - joint_limit_buffers

device = torch.device('cpu')


class CEPPolicy():
    def __init__(self, dt=1 / 240., dtype='float64'):
        self.dt = dt
        self.dtype = dtype

        self.controller = cep_simple_model_tiago()

    def policy(self, state):
        joint_poses = state[0, 0:7]
        joint_vels = state[0, 7:]

        action = self.controller.policy(state)

        x, dx = self.step(joint_poses, joint_vels, action)
        return x, dx


    def step(self, joint_poses, joint_vels, joint_accs):
        joint_poses = joint_poses + joint_vels * self.dt
        joint_vels = joint_vels + joint_accs * self.dt
        return joint_poses, joint_vels


def experiment():
    '''
    envlist:[it can take values [1,2,3]. It provides the type of environment we are using]
    results_dir: path to the folder in which we are saving the results
    '''

    time_step = 1 / 250.

    env = TiagoOneParallelHand(time_step=time_step)

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

        for i in range(horizon):
            init = time.time()

            #### Get Control Action (Position Control)####
            a = policy.policy(state)
            state, reward, done, success = env.step(a)
            #############################

            end = time.time()
            time.sleep(np.clip(time_step - (end - init), 0, time_step))

            if i == (horizon-1):
                REWARD = reward
                END_POSITION = env.check_endPosition()
        print('Reward:',  REWARD)
        print('End position: ', END_POSITION)
        print('Desired position', env.Target_pos)
    p.disconnect()


if __name__ == '__main__':
    p.connect(p.GUI_SERVER, 1234,
              options='--background_color_red=1. --background_color_green=1. --background_color_blue=1.')
    p.resetDebugVisualizerCamera(2.2, 55.6, -47.4, [0.04, 0.06, 0.31])
    experiment()

