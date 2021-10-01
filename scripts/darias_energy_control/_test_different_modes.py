import numpy as np
import pybullet as p
import time
import matplotlib.pyplot as plt

from cep.envs import Tiago_LeftParallelHand_Base
from cep.cep_models import cep_tiago_pathplan_trackfather
from cep.cep_models import cep_tiago_lefthand_base_pathplan
import torch
from mat_animation import Plotting

joint_limit_buffers = 0.02
joint_limits = np.array([2.5, 2.5, 3.1416, 2.75, 1.57, 3.53, 2.35, 2.09, 1.57, 2.09]) - joint_limit_buffers
BASE_THRESHOLD = 0.01

device = torch.device('cpu')

class CEP_Policy:
    def __init__(self, dt=1/240., dtype='float64', cep_model=None):
        self.dt = dt
        self.dtype = dtype

        self.controller = cep_model

    def policy(self, state):
        joint_poses = state[0, :2]
        joint_vels = state[0, 10:12]

        action = self.controller.policy(state)

        x, dx = self.step(joint_poses, joint_vels, action[:2])
        return x, dx

    def step(self, joint_poses, joint_vels, joint_accs):
        joint_vels = joint_vels + joint_accs * self.dt
        joint_poses = joint_poses + joint_vels * self.dt  # TODO: WHY??? 08.18
        return joint_poses, joint_vels

def experiment(CEP_Policy):

    """
    envlist:[it can take values [1,2,3]. It provides the type of environment we are using]
    results_dir: path to the folder in which we are saving the results
    """

    print("Current CEP model: %s" % (CEP_Policy))

    time_step = 1 / 240.

    env = Tiago_LeftParallelHand_Base(time_step=time_step, Target_pose=[1.5, 1.2, 0.8])

    cur_policy = CEP_Policy
    ################

    n_trials = 100
    horizon = 6000
    c = 0
    s = 0
    END_POSITION = None
    for itr in range(n_trials):
        print('###Iteration: {}'.format(itr))
        state = env.reset()
        p.addUserDebugLine([0., 0., -0.189], [1.5, 0., -0.189], [1., 0., 0.])

        robot_x_list = []
        robot_y_list = []
        dist_list = []

        FLAG_FIRS_TIME_CHECK_COL = True
        FLAG_FIRS_TIME_CHECK_SUC = True
        FLAG_FIRS_TIME_CHECK_DONE = True
        SUCCESS = 0
        COLLISION = 0
        DONE = False
        TIME_USED = 0.

        start_exp = time.time()
        for i in range(horizon):
            #print("### iteration: ", i)
            init = time.time()
            #### Get Control Action (Position Control)####
            a = cur_policy.policy(state)
            state, base_dist, done, success, col = env.step(a)
            ###################################

            # TODO: Check collision and success | 09.29
            if FLAG_FIRS_TIME_CHECK_COL:
                if col:
                    COLLISION = 1
                    FLAG_FIRS_TIME_CHECK_COL = False
            if FLAG_FIRS_TIME_CHECK_SUC:
                if success:
                    SUCCESS = 1
                    FLAG_FIRS_TIME_CHECK_SUC = False
            if FLAG_FIRS_TIME_CHECK_DONE:
                if not COLLISION and SUCCESS:
                    DONE = 1
                    FLAG_FIRS_TIME_CHECK_DONE = False
            ###################################

            # TODO: Record robot x and y values | 09.16
            robot_x_list.append(state[0][0])
            robot_y_list.append(state[0][1])
            dist_list.append(base_dist)
            ###################################

            end = time.time()
            time.sleep(np.clip(time_step - (end - init), 0, time_step))

            if i == (horizon-1) or base_dist <= BASE_THRESHOLD:
                end_exp = time.time()
                TIME_USED = end_exp - start_exp
                Distance = base_dist
                ITER_TIMES = i

                END_POSITION = env.check_endPosition()
                #print('Position state: ', state[0])
                print('Distance:',  Distance)
                print('End position: ', END_POSITION)
                #print('Desired position', env.Target_pos)
                print("### SUCCESS: ", SUCCESS)
                print("### COLLISION: ", COLLISION)
                print("### DONE", DONE)
                print("### TIME_USED: ", TIME_USED)
                print("### ITER_TIMES: ", ITER_TIMES)
                return SUCCESS, COLLISION, TIME_USED, ITER_TIMES
                ##################################
                # # TODO: Matplot animation version | 09.16
                # print("len(robot_x_list): ", len(robot_x_list))
                # print("robot_x_list: ", robot_x_list)
                # print("robot_y_list: ", robot_y_list)
                # plotting = Plotting(robot_x_list=robot_x_list, robot_y_list=robot_y_list, dist_list=dist_list, horizon=i)
                # plotting.plot_fig()
                # plotting.plot_animation()
                ##################################
                break
    #p.disconnect()

def plot(success_list, collision_list, time_used_list, iter_list, test_cep_models):  # TODO: Add on 09.29

    test_cep_models = ['track father', 'cascade control']
    num = len(success_list)  # Number of policies

    fig, axs = plt.subplots(1, num)

    # the histogram of the data
    for i in range(num):
        cur_suc = success_list[i]
        len_suc = float(len(cur_suc))
        suc = float(sum(cur_suc))
        cur_col = collision_list[i]
        col = float(sum(cur_col))
        len_col = float(len(cur_col))
        cur_iter = iter_list[i]
        mean_iter = np.mean(cur_iter)
        height = [suc / len_suc, col / len_col, mean_iter/10000.]

        ax = axs[i]
        label = ['success', 'collision', 'iteration times']
        colors = ['tan', 'lime', 'red']
        x = np.arange(len(label))
        for j in range(len(label)):
            ax.bar(x[j], height=height[j], color=colors[j], label=label[j])
        ax.legend(prop={'size': 10})
        ax.set_ylim(0., 1.)
        ax.set_xlabel('success | collision | iteration')
        ax.set_ylabel('Success rate, collision rate and iteration times')
        ax.set_title("{}".format(test_cep_models[i]))

    # Tweak spacing to prevent clipping of ylabel
    fig.tight_layout()
    plt.show()

def plot_suc_col(success_list, collision_list, time_used_list, iter_list, test_cep_models): # TODO: added | 09.30

    test_cep_models = ['cascade_control', 'track_father']
    label = ['success rate', 'collision rate', 'iteration times']
    x = np.arange(2)  # the label locations
    width = 0.32  # the width of the bars
    w = 0.16
    height_suc = []
    height_col = []
    height_iter = []
    for i in range(2):
        cur_suc = success_list[i]
        len_suc = float(len(cur_suc))
        suc = float(sum(cur_suc))
        height_suc.append(suc / len_suc)
        cur_col = collision_list[i]
        col = float(sum(cur_col))
        len_col = float(len(cur_col))
        height_col.append(col / len_col)
        cur_iter = iter_list[i]
        mean_iter = np.mean(cur_iter)
        height_iter.append(mean_iter / 10000.)

    fig, ax = plt.subplots()
    rects1 = ax.bar(x - width / 2, height_suc, w, label=label[0])
    rects2 = ax.bar(x, height_col, w, label=label[1])
    rects3 = ax.bar(x + width / 2, height_iter, w, label=label[2])

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel('Success rate, collision rate and iteration times')
    ax.set_title('Comparison of different CEP models')
    ax.set_xticks(x)
    ax.set_xticklabels(test_cep_models)
    ax.legend()

    ax.bar_label(rects1, padding=3)
    ax.bar_label(rects2, padding=3)
    ax.bar_label(rects3, padding=3)
    fig.tight_layout()

    plt.show()


def compare_experiments(test_cep_policies, test_cep_models, exp_times):

    success_list = []
    collision_list = []
    time_used_list = []
    iter_list = []

    for policy in test_cep_policies:
        cur_success_list = []
        cur_collision_list = []
        cur_time_used_list = []
        cur_iter_list = []
        for exp in range(exp_times):
            cur_suc, cur_col, cur_time, cur_iter = experiment(policy)
            cur_success_list.append(cur_suc)
            cur_collision_list.append(cur_col)
            cur_time_used_list.append(cur_time)
            cur_iter_list.append(cur_iter)

        success_list.append(cur_success_list)
        collision_list.append(cur_collision_list)
        time_used_list.append(cur_time_used_list)
        iter_list.append(cur_iter_list)


    #######################################
    # TODO: Draw histogram for comparing different policy | 09.29
    plot_suc_col(success_list, collision_list, time_used_list, iter_list, test_cep_models)
    #######################################


test_cep_models = [cep_tiago_lefthand_base_pathplan(), cep_tiago_pathplan_trackfather()]
test_cep_policies = []
for model in test_cep_models:
    test_cep_policies.append(CEP_Policy(cep_model=model))
p.connect(p.DIRECT, 1234,
          options='--background_color_red=1. --background_color_green=1. --background_color_blue=1.')
p.resetDebugVisualizerCamera(2.2, 55.6, -47.4, [0.04, 0.06, 0.31])
compare_experiments(test_cep_policies, test_cep_models, exp_times=10)

