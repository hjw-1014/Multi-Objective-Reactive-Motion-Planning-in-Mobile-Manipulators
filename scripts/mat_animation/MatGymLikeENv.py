"""
    Tiago Gym-like Env using Matlibplot
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from matplotlib import animation, rc
from IPython.display import HTML, Image
import math

# Define robot configuration
radius = 0.28
robot_x = 0.
robot_y = 0.

class MatTiagoEnv:

    def __init__(self, delta=1./50., acc_control=False, enable_apf=False, start_point=None):

        self.obstacle = None
        self.START_POINT = start_point
        #self.START_POINT = np.array([-0.1, -0.1])
        self.END_POINT = np.array([1.2, 1.0])
        self.BOUND_X = np.array([0.07, 0.93])  # np.array([[X_MIN, X_MAX]])
        self.BOUND_Y = np.array([0.07, 0.93])
        self.BOX_CENTER = np.array([0.5, 0.5])
        self.robot_radius = 0.28
        self.vel_init = np.array([0., 0.])

        self.cur_pos = None
        self.cur_vel = None

        self.delta = delta
        self.THRESHOLD = 0.03

        self.acc_ctl = acc_control
        self.apf = enable_apf

    def reset(self):

        self.cur_pos = self.START_POINT
        self.cur_vel = self.vel_init

        if not self.apf:
        # TODO: State is position and velocity
            init_state = np.concatenate([self.cur_pos, self.cur_vel]).reshape(-1, 1)

        elif self.apf:
        # TODO: State is position
            init_state = self.cur_pos

        return init_state

    def step(self, action: np.array):
        """
            Dynamics: Given a Action (acceleration or velocity), return the next state (position and velocity)
        """

        # TODO: Action is accelerationc
        if self.acc_ctl:
            alpha = 0.3
            cur_vel = self.cur_vel + self.delta * action
            cur_pos = self.cur_pos + self.delta * cur_vel
            obs = np.concatenate([cur_pos, cur_vel])
            self.cur_vel = cur_vel
            self.cur_pos = cur_pos

        # TODO: Action is velocity
        else:
            alpha = 0.7
            cur_pos = self.cur_pos + self.delta * action
            self.cur_pos = alpha * cur_pos + (1 - alpha) * self.cur_pos
            obs = self.cur_pos

        dist = self.cal_dist()
        done = self.check_arrive()
        success = self.check_success()
        col = self.check_collision()

        return obs, dist, done, success, col

    def check_success(self):
        if self.check_arrive() and not self.check_collision():
            return True
        return False

    def check_arrive(self):
        if math.fabs(self.cal_dist()) <= self.THRESHOLD:
            return True
        return False

    def check_collision(self):
        #if self.BOUND_X[0] <= self.cur_pos[0] <= self.BOUND_X[1] and self.BOUND_Y[0] <= self.cur_pos[1] <= self.BOUND_Y[1]:
        if math.hypot(self.cur_pos[0]-self.BOX_CENTER[0], self.cur_pos[1]-self.BOX_CENTER[1]) <= 0.43:
            return True
        return False

    def check_reward(self):

        return 0

    def cal_dist(self):
        return math.hypot(self.cur_pos[0] - self.END_POINT[0], self.cur_pos[1] - self.END_POINT[1])


class Plotting:
    def __init__(self, robot_x_list=None, robot_y_list=None, robot_z_list=None,dist_list=None, horizon=None):

        self.robot = None
        self.robot_x_list = robot_x_list
        self.robot_y_list = robot_y_list
        self.robot_z_list = robot_z_list
        self.horizon = horizon
        self.dist_list = dist_list

    def plot_robot(self, ax, robot_x, robot_y, radius):

        robot_color = 'blue'
        robot_circle = plt.Circle((robot_x, robot_y), radius, color=robot_color,  fill=False)
        ax.add_patch(robot_circle)

    def plot_des(self, ax, des_x, des_y, radius):

        desire_cicle = plt.Circle((des_x, des_y), radius, color='g', fill=False)
        ax.add_patch(desire_cicle)

    def plot_animation(self):

        fig, ax = plt.subplots(figsize=(9, 9))

        robot_x_list = self.robot_x_list
        robot_y_list = self.robot_y_list

        for _ in range(10):
            for i in range(0, self.horizon, int(self.horizon/75)):

                plt.cla()

                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None])
                ax.set_xlim(-1, 2)
                ax.set_ylim(-1, 2)

                ax.grid(True)
                ax.grid(which='major', alpha=0.5)

                self.plot_robot(ax, robot_x_list[i], robot_y_list[i], radius)

                robot_circle = plt.Circle((robot_x_list[i], robot_y_list[i]), radius=0.01, color='y', fill=True)
                ax.text(robot_x_list[i], robot_y_list[i], s='cur', fontsize=8.)
                ax.add_patch(robot_circle)

                des_x = 1.2
                des_y = 1.0
                self.plot_des(ax, des_x, des_y, radius)

                start_circle = plt.Circle((-0.1, -0.1), 0.01, color='g', fill=True)
                ax.text(-0.1, -0.1, s='start', fontsize=8.)
                ax.add_patch(start_circle)

                goal_circle = plt.Circle((1.2, 1.0), 0.01, color='r', fill=True)
                ax.text(1.2, 1.0, s='goal', fontsize=8.)
                ax.add_patch(goal_circle)

                # box_circle = plt.Circle((0.5, 0.5), 0.15, color='r', fill=True)
                # ax.text(0.5, 0.5, s='Box', fontsize=8.)
                # ax.add_patch(box_circle)
                ax.add_patch(Rectangle((0.35, 0.35), 0.3, 0.3, facecolor="black", alpha=0.5))

                plt.pause(1e-2)

                #l = data_linewidth_plot([0., 0.2], [0., 0.2], ax=ax, label='some 1 data unit wide line',
                #                    linewidth=0.01, alpha=1.)

            #plt.show()

    def plot_xy(self, des_x=1.2, des_y=1.0, des_z=0.):

        fig, ax = plt.subplots(1, 3, figsize=(9, 9))
        l = len(self.robot_x_list)
        x = np.linspace(0, self.horizon, l)
        ax[0].plot(x, self.robot_x_list)
        ax[0].set_title("Position X")
        ax[0].plot(x, np.ones(len(self.robot_x_list)) * des_x)
        #ax[0].legend("Distance /m")
        ax[1].plot(x, self.robot_y_list)
        ax[1].set_title("Position Y")
        ax[1].plot(x, np.ones(len(self.robot_x_list)) * des_y)
        ax[2].plot(x, self.dist_list)
        ax[2].plot(x, np.zeros(len(self.robot_x_list)))
        ax[2].set_title("Distance from goal")
        plt.show()

    def plot_fig_wholebody(self, des_x=1.7, des_y=1.1, des_z=0.8):
        fig, ax = plt.subplots(1, 4, figsize=(9, 9))
        l = len(self.robot_x_list)
        x = np.linspace(0, self.horizon, l)
        ax[0].plot(x, self.robot_x_list)
        ax[0].set_title("Position X")
        ax[0].plot(x, np.ones(len(self.robot_x_list)) * des_x)
        # ax[0].legend("Distance /m")
        ax[1].plot(x, self.robot_y_list)
        ax[1].set_title("Position Y")
        ax[1].plot(x, np.ones(len(self.robot_x_list)) * des_y)
        ax[2].plot(x, self.robot_z_list)
        ax[2].set_title("Position Z")
        ax[2].plot(x, np.ones(len(self.robot_x_list)) * des_z)
        ax[3].plot(x, self.dist_list)
        ax[3].plot(x, np.zeros(len(self.robot_x_list)))
        ax[3].set_title("Distance from goal")
        plt.show()

    def animate(self, i, ax):

        line, = ax.plot([], [], lw=2)
        x = np.linspace(0, 2, 1000)
        y = np.sin(2 * np.pi * (x - 0.01 * i))
        line.set_data(x, y)

        return (line,)

    def save_gif(self, fig, ax):

        anim = animation.FuncAnimation(fig, self.animate(ax),
                                       frames=100, interval=20, blit=True)
        anim.save("test.gif", writer='imagemagick', fps=60)

    def plot_path(self, robot_x_list, robot_y_list):

        fig, ax = plt.subplots()
        center_size = 10
        marker_size = 5000

        for i in range(0, len(robot_x_list), 100):
            ax.scatter(robot_x_list[i], robot_y_list[i], s=center_size, alpha=0.7)
            ax.scatter(robot_x_list[i], robot_y_list[i], s=marker_size, alpha=.04)

        ax.text(0., 0., "Start")
        ax.text(1.2, 1., "End")
        ax.set_title("CEP with tracking_father")
        #ax.set_title("CEP with cascade_control")

        ax.grid(True)

        ax.set_xlim(-0.5, 1.5)
        ax.set_ylim(-0.5, 1.5)

        box_center_x = 0.5
        box_center_y = 0.5
        ax.text(box_center_x, box_center_y, "Box")
        #
        left, bottom, width, height = (0.35, 0.35, 0.3, 0.3)
        ax.add_patch(Rectangle((left, bottom), width, height,
                               facecolor="black", alpha=0.5))

        # box_circle = plt.Circle((0.5, 0.5), 0.15, color='r', fill=True)
        # ax.text(0.5, 0.5, s='Box', fontsize=8.)
        # ax.add_patch(box_circle)

        # t = time.strftime("%Y-%m-%d-%H_%M_%S", time.localtime())
        # myfig = plt.gcf()
        # myfig.savefig('figures/run_exp_{}.png'.format(t), dpi=300)

        plt.show()


class MatTiagoEnvComplex:

    def __init__(self, delta=1./50., acc_control=False, enable_apf=False, start_point=None):

        self.collision_threshold = 0.2
        self.obstacle = None
        self.START_POINT = start_point
        #self.START_POINT = np.array([-0.1, -0.1])
        self.END_POINT = np.array([1.2, 1.2])

        self.OBS_CIRCLE_RADIUS = 0.1
        self.OBS_REC_EDGE = 0.2
        self.OBS_CIRCLE_1 = np.array([0.3, 0.25])
        self.OBS_CIRCLE_2 = np.array([0.6, 0.7])
        self.OBS_CIRCLE_3 = np.array([0.1, 0.9])
        self.OBS_RECTANGLE_1 = np.array([1.0, 0.2])
        self.OBS_RECTANGLE_1_BOUND_X = np.array([0.9, 1.1])
        self.OBS_RECTANGLE_1_BOUND_Y = np.array([0.1, 0.3])

        # self.BOUND_X = np.array([0.07, 0.93])  # np.array([[X_MIN, X_MAX]])
        # self.BOUND_Y = np.array([0.07, 0.93])
        # self.BOX_CENTER = np.array([0.5, 0.5])
        self.robot_radius = 0.1
        self.vel_init = np.array([0., 0.])

        self.cur_pos = None
        self.cur_vel = None

        self.delta = delta
        self.THRESHOLD = 0.03

        self.acc_ctl = acc_control
        self.apf = enable_apf

    def reset(self):

        self.cur_pos = self.START_POINT
        self.cur_vel = self.vel_init

        if not self.apf:
        # TODO: State is position and velocity
            init_state = np.concatenate([self.cur_pos, self.cur_vel]).reshape(-1, 1)

        elif self.apf:
        # TODO: State is position
            init_state = self.cur_pos

        return init_state

    def step(self, action: np.array):
        """
            Dynamics: Given a Action (acceleration or velocity), return the next state (position and velocity)
        """

        # TODO: Action is accelerationc
        if self.acc_ctl:
            alpha = 0.3
            cur_vel = self.cur_vel + self.delta * action
            cur_pos = self.cur_pos + self.delta * cur_vel
            obs = np.concatenate([cur_pos, cur_vel])
            self.cur_vel = cur_vel
            self.cur_pos = cur_pos

        # TODO: Action is velocity
        else:
            alpha = 0.7
            cur_pos = self.cur_pos + self.delta * action
            self.cur_pos = alpha * cur_pos + (1 - alpha) * self.cur_pos
            obs = self.cur_pos

        dist = self.cal_dist()
        done = self.check_arrive()
        success = self.check_success()
        col = self.check_collision()

        return obs, dist, done, success, col

    def check_success(self):
        if self.check_arrive() and not self.check_collision():
            return True
        return False

    def check_arrive(self):
        if math.fabs(self.cal_dist()) <= self.THRESHOLD:
            return True
        return False

    def check_collision(self):
        #if self.BOUND_X[0] <= self.cur_pos[0] <= self.BOUND_X[1] and self.BOUND_Y[0] <= self.cur_pos[1] <= self.BOUND_Y[1]:
        if math.hypot(self.cur_pos[0]-self.OBS_CIRCLE_1[0], self.cur_pos[1]-self.OBS_CIRCLE_1[1]) <= self.collision_threshold or \
            math.hypot(self.cur_pos[0] - self.OBS_CIRCLE_2[0], self.cur_pos[1] - self.OBS_CIRCLE_2[1]) <=  self.collision_threshold or \
            math.hypot(self.cur_pos[0] - self.OBS_CIRCLE_3[0], self.cur_pos[1] - self.OBS_CIRCLE_3[1]) <= self.collision_threshold or \
                (self.cur_pos[0] >= self.OBS_RECTANGLE_1_BOUND_X[0] and self.cur_pos[0] <= self.OBS_RECTANGLE_1_BOUND_X[1] and \
                 self.cur_pos[1] >= self.OBS_RECTANGLE_1_BOUND_Y[0] and self.cur_pos[1] <= self.OBS_RECTANGLE_1_BOUND_Y[1]):
            return True
        return False

    def check_reward(self):

        return 0

    def cal_dist(self):
        return math.hypot(self.cur_pos[0] - self.END_POINT[0], self.cur_pos[1] - self.END_POINT[1])

class PlottingComplex:
    def __init__(self, robot_x_list=None, robot_y_list=None, robot_z_list=None,dist_list=None, horizon=None):

        self.robot_radius = 0.10
        self.des_x = 1.2
        self.des_y = 1.2
        self.robot = None
        self.robot_x_list = robot_x_list
        self.robot_y_list = robot_y_list
        self.robot_z_list = robot_z_list
        self.horizon = horizon
        self.dist_list = dist_list

    def plot_robot(self, ax, robot_x, robot_y, radius):

        robot_color = 'blue'
        robot_circle = plt.Circle((robot_x, robot_y), self.robot_radius, color=robot_color,  fill=False)
        ax.add_patch(robot_circle)

    def plot_des(self, ax, des_x, des_y, radius):

        desire_cicle = plt.Circle((des_x, des_y), self.robot_radius, color='g', fill=False)
        ax.add_patch(desire_cicle)

    def plot_animation(self):

        fig, ax = plt.subplots(figsize=(9, 9))

        robot_x_list = self.robot_x_list
        robot_y_list = self.robot_y_list

        for _ in range(10):
            for i in range(0, self.horizon, int(self.horizon/100)):

                plt.cla()

                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None])
                ax.set_xlim(-1, 2)
                ax.set_ylim(-1, 2)

                ax.grid(True)
                ax.grid(which='major', alpha=0.5)

                self.plot_robot(ax, robot_x_list[i], robot_y_list[i], self.robot_radius)

                robot_circle = plt.Circle((robot_x_list[i], robot_y_list[i]), radius=0.01, color='y', fill=True)
                ax.text(robot_x_list[i], robot_y_list[i], s='cur', fontsize=8.)
                ax.add_patch(robot_circle)

                des_x = self.des_x
                des_y = self.des_y
                self.plot_des(ax, des_x, des_y, self.robot_radius)

                start_circle = plt.Circle((-0.1, -0.1), 0.01, color='g', fill=True)
                ax.text(-0.1, -0.1, s='start', fontsize=9.)
                ax.add_patch(start_circle)

                goal_circle = plt.Circle((self.des_x, self.des_y), 0.01, color='r', fill=True)
                ax.text(self.des_x, self.des_y, s='goal', fontsize=9.)
                ax.add_patch(goal_circle)

                self.OBS_CIRCLE_1 = np.array([0.3, 0.25])
                self.OBS_CIRCLE_2 = np.array([0.6, 0.7])
                self.OBS_CIRCLE_3 = np.array([0.1, 0.9])
                self.OBS_RECTANGLE_1 = np.array([1.0, 0.2])

                obs_circle_1 = plt.Circle((0.3, 0.25), 0.1, color='b', fill=True)
                ax.add_patch(obs_circle_1)
                obs_circle_2 = plt.Circle((0.6, 0.7), 0.1, color='b', fill=True)
                ax.add_patch(obs_circle_2)
                obs_circle_3 = plt.Circle((0.1, 0.9), 0.1, color='b', fill=True)
                ax.add_patch(obs_circle_3)
                ax.add_patch(Rectangle((1.0, 0.2), 0.2, 0.2, facecolor="black", alpha=0.5))

                plt.pause(1e-2)

                #l = data_linewidth_plot([0., 0.2], [0., 0.2], ax=ax, label='some 1 data unit wide line',
                #                    linewidth=0.01, alpha=1.)

            #plt.show()

    def plot_xy(self, des_x=1.2, des_y=1.2, des_z=0.):

        fig, ax = plt.subplots(1, 3, figsize=(9, 9))
        l = len(self.robot_x_list)
        x = np.linspace(0, self.horizon, l)
        ax[0].plot(x, self.robot_x_list)
        ax[0].set_title("Position X")
        ax[0].plot(x, np.ones(len(self.robot_x_list)) * des_x)
        #ax[0].legend("Distance /m")
        ax[1].plot(x, self.robot_y_list)
        ax[1].set_title("Position Y")
        ax[1].plot(x, np.ones(len(self.robot_x_list)) * des_y)
        ax[2].plot(x, self.dist_list)
        ax[2].plot(x, np.zeros(len(self.robot_x_list)))
        ax[2].set_title("Distance from goal")
        plt.show()

    def plot_fig_wholebody(self, des_x=1.7, des_y=1.1, des_z=0.8):
        fig, ax = plt.subplots(1, 4, figsize=(9, 9))
        l = len(self.robot_x_list)
        x = np.linspace(0, self.horizon, l)
        ax[0].plot(x, self.robot_x_list)
        ax[0].set_title("Position X")
        ax[0].plot(x, np.ones(len(self.robot_x_list)) * des_x)
        # ax[0].legend("Distance /m")
        ax[1].plot(x, self.robot_y_list)
        ax[1].set_title("Position Y")
        ax[1].plot(x, np.ones(len(self.robot_x_list)) * des_y)
        ax[2].plot(x, self.robot_z_list)
        ax[2].set_title("Position Z")
        ax[2].plot(x, np.ones(len(self.robot_x_list)) * des_z)
        ax[3].plot(x, self.dist_list)
        ax[3].plot(x, np.zeros(len(self.robot_x_list)))
        ax[3].set_title("Distance from goal")
        plt.show()

    def animate(self, i, ax):

        line, = ax.plot([], [], lw=2)
        x = np.linspace(0, 2, 1000)
        y = np.sin(2 * np.pi * (x - 0.01 * i))
        line.set_data(x, y)

        return (line,)

    def save_gif(self, fig, ax):

        anim = animation.FuncAnimation(fig, self.animate(ax),
                                       frames=100, interval=20, blit=True)
        anim.save("test.gif", writer='imagemagick', fps=60)

    def plot_path(self, robot_x_list, robot_y_list):

        fig, ax = plt.subplots()
        center_size = 10
        marker_size = 1000

        for i in range(0, len(robot_x_list), 20):
            ax.scatter(robot_x_list[i], robot_y_list[i], s=center_size, alpha=0.7)
            ax.scatter(robot_x_list[i], robot_y_list[i], s=marker_size, alpha=.04)

        ax.set_title("2D path planning")
        #ax.set_title("CEP with cascade_control")

        start_circle = plt.Circle((-0.1, -0.1), 0.01, color='g', fill=True)
        ax.text(-0.1, -0.1, s='start', fontsize=9.)
        ax.add_patch(start_circle)

        goal_circle = plt.Circle((self.des_x, self.des_y), 0.01, color='r', fill=True)
        ax.text(self.des_x, self.des_y, s='goal', fontsize=9.)
        ax.add_patch(goal_circle)

        ax.grid(True)

        ax.set_xlim(-0.5, 1.5)
        ax.set_ylim(-0.5, 1.5)

        # box_center_x = 0.5
        # box_center_y = 0.5
        # ax.text(box_center_x, box_center_y, "Box")

        obs_circle_1 = plt.Circle((0.3, 0.25), 0.1, color='b', fill=True)
        ax.add_patch(obs_circle_1)
        obs_circle_2 = plt.Circle((0.6, 0.7), 0.1, color='b', fill=True)
        ax.add_patch(obs_circle_2)
        obs_circle_3 = plt.Circle((0.1, 0.9), 0.1, color='b', fill=True)
        ax.add_patch(obs_circle_3)
        ax.add_patch(Rectangle((1.0, 0.2), 0.2, 0.2, facecolor="black", alpha=0.5))

        # left, bottom, width, height = (0.35, 0.35, 0.3, 0.3)
        # ax.add_patch(Rectangle((left, bottom), width, height,
        #                        facecolor="black", alpha=0.5))

        # t = time.strftime("%Y-%m-%d-%H_%M_%S", time.localtime())
        # myfig = plt.gcf()
        # myfig.savefig('figures/run_exp_{}.png'.format(t), dpi=300)

        plt.show()

# class Plotting:
#     def __init__(self, robot_x_list=None, robot_y_list=None, dist_list=None, horizon=None):
#
#         self.robot = None
#         self.radius = 0.28
#         self.des_x = 1.2
#         self.des_y = 1.0
#         self.robot_x_list = robot_x_list
#         self.robot_y_list = robot_y_list
#         self.horizon = horizon
#         self.dist_list = dist_list
#
#     def plot_robot(self, ax, robot_x, robot_y, radius):
#
#         robot_color = 'blue'
#         robot_circle = plt.Circle((robot_x, robot_y), radius, color=robot_color,  fill=False)
#         ax.add_patch(robot_circle)
#
#     def plot_des(self, ax, des_x, des_y, radius):
#
#         desire_cicle = plt.Circle((des_x, des_y), radius, color='g', fill=False)
#         ax.add_patch(desire_cicle)
#
#     def plot_animation(self):
#
#         fig, ax = plt.subplots(figsize=(9, 9))
#
#         robot_x_list = self.robot_x_list
#         robot_y_list = self.robot_y_list
#
#         for _ in range(2):
#             for i in range(0, self.horizon, int(self.horizon/100)):
#
#                 plt.cla()
#
#                 # for stopping simulation with the esc key.
#                 plt.gcf().canvas.mpl_connect('key_release_event',
#                                              lambda event: [exit(0) if event.key == 'escape' else None])
#                 ax.set_xlim(-1, 2)
#                 ax.set_ylim(-1, 2)
#
#                 ax.grid(True)
#
#                 self.plot_robot(ax, robot_x_list[i], robot_y_list[i], self.radius)
#
#                 robot_circle = plt.Circle((robot_x_list[i], robot_y_list[i]), radius=0.01, color='y', fill=True)
#                 ax.text(robot_x_list[i], robot_y_list[i], s='cur', fontsize=8.)
#                 ax.add_patch(robot_circle)
#
#                 self.plot_des(ax, self.des_x, self.des_y, self.radius)
#
#                 start_circle = plt.Circle((-0.1, -0.1), 0.01, color='g', fill=True)
#                 ax.text(-0.1, -0.1, s='start', fontsize=8.)
#                 ax.add_patch(start_circle)
#
#                 goal_circle = plt.Circle((1.2, 1.0), 0.01, color='r', fill=True)
#                 ax.text(1.2, 1.0, s='goal', fontsize=8.)
#                 ax.add_patch(goal_circle)
#
#                 ax.add_patch(Rectangle((0.35, 0.35), 0.3, 0.3, facecolor="black", alpha=0.5))
#
#                 plt.pause(1e-1)
#
#                 #l = data_linewidth_plot([0., 0.2], [0., 0.2], ax=ax, label='some 1 data unit wide line',
#                 #                    linewidth=0.01, alpha=1.)
#
#             #plt.show()
#
#     def plot_fig(self):
#
#         fig, ax = plt.subplots(1, 3, figsize=(9, 9))
#         l = len(self.robot_x_list)
#         x = np.linspace(0, self.horizon, l)
#         ax[0].plot(x, self.robot_x_list)
#         ax[0].set_title("Base position X")
#         ax[1].plot(x, self.robot_y_list)
#         ax[1].set_title("Base position Y")
#         ax[2].plot(x, self.dist_list)
#         ax[2].set_title("Base distance from goal")
#         plt.show()
#
#     def animate(self, i, ax):
#
#         line, = ax.plot([], [], lw=2)
#         x = np.linspace(0, 2, 1000)
#         y = np.sin(2 * np.pi * (x - 0.01 * i))
#         line.set_data(x, y)
#
#         return (line,)
#
#     def save_gif(self, fig, ax):
#
#         anim = animation.FuncAnimation(fig, self.animate(ax),
#                                        frames=100, interval=20, blit=True)
#         anim.save("test.gif", writer='imagemagick', fps=60)