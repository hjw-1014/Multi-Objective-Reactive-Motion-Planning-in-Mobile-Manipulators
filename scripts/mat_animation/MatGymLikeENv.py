"""
    Tiago Gym-like Env using Matlibplot
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from matplotlib import animation, rc
from IPython.display import HTML, Image
import math


class MatTiagoEnv:

    def __init__(self, delta=1./50.):

        self.obstacle = None
        self.START_POINT = np.array([-0.1, -0.1])
        self.END_POINT = np.array([1.2, 1.0])
        self.BOUND_X = np.array([0.07, 0.93])  # np.array([[X_MIN, X_MAX]])
        self.BOUND_Y = np.array([0.07, 0.93])
        self.robot_radius = 0.28
        self.vel_init = np.array([0., 0.])

        self.cur_pos = None
        self.cur_vel = None

        self.delta = delta
        self.THRESHOLD = 0.05

    def reset(self):

        self.cur_pos = self.START_POINT
        self.cur_vel = self.vel_init

        # TODO: State is position and velocity
        # init_state =  np.concatenate([self.cur_pos, self.cur_vel]).reshape(-1, 1)

        # TODO: State is position
        init_state = self.cur_pos

        return init_state

    def step(self, action: np.array):
        """
            Dynamics: Given a Action (acceleration or velocity), return the next state (position and velocity)
        """

        # TODO: Action is acceleration
        # self.cur_vel += self.delta * action
        # self.cur_pos += self.delta * self.cur_vel
        # obs = np.concatenate([self.cur_pos, self.cur_vel])

        # TODO: Action is velocity
        self.cur_pos += self.delta * action
        obs = self.cur_pos

        reward = self.check_reward()
        dist = self.cal_dist()
        done = self.check_done()
        col = self.check_collision()



        return obs, dist, reward, done, col

    def check_done(self):
        if self.check_arrive() and not self.check_collision():
            return True
        return False

    def check_arrive(self):
        if math.fabs(self.cal_dist()) <= self.THRESHOLD:
            return True
        return False

    def check_collision(self):
        if self.BOUND_X[0] <= self.cur_pos[0] <= self.BOUND_X[1] and self.BOUND_Y[0] <= self.cur_pos[1] <= self.BOUND_Y[1]:
            return True
        return False

    def check_reward(self):

        return 0

    def cal_dist(self):
        return math.hypot(self.cur_pos[0] - self.END_POINT[0], self.cur_pos[1] - self.END_POINT[1])


class Plotting:
    def __init__(self, robot_x_list=None, robot_y_list=None, dist_list=None, horizon=None):

        self.robot = None
        self.radius = 0.28
        self.des_x = 1.2
        self.des_y = 1.0
        self.robot_x_list = robot_x_list
        self.robot_y_list = robot_y_list
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

        for _ in range(2):
            for i in range(0, self.horizon, int(self.horizon/100)):

                plt.cla()

                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None])
                ax.set_xlim(-1, 2)
                ax.set_ylim(-1, 2)

                ax.grid(True)

                self.plot_robot(ax, robot_x_list[i], robot_y_list[i], self.radius)

                robot_circle = plt.Circle((robot_x_list[i], robot_y_list[i]), radius=0.01, color='y', fill=True)
                ax.text(robot_x_list[i], robot_y_list[i], s='cur', fontsize=8.)
                ax.add_patch(robot_circle)

                self.plot_des(ax, self.des_x, self.des_y, self.radius)

                start_circle = plt.Circle((-0.1, -0.1), 0.01, color='g', fill=True)
                ax.text(-0.1, -0.1, s='start', fontsize=8.)
                ax.add_patch(start_circle)

                goal_circle = plt.Circle((1.2, 1.0), 0.01, color='r', fill=True)
                ax.text(1.2, 1.0, s='goal', fontsize=8.)
                ax.add_patch(goal_circle)

                ax.add_patch(Rectangle((0.35, 0.35), 0.3, 0.3, facecolor="black", alpha=0.5))

                plt.pause(1e-1)

                #l = data_linewidth_plot([0., 0.2], [0., 0.2], ax=ax, label='some 1 data unit wide line',
                #                    linewidth=0.01, alpha=1.)

            #plt.show()

    def plot_fig(self):

        fig, ax = plt.subplots(1, 3, figsize=(9, 9))
        l = len(self.robot_x_list)
        x = np.linspace(0, self.horizon, l)
        ax[0].plot(x, self.robot_x_list)
        ax[0].set_title("Base position X")
        ax[1].plot(x, self.robot_y_list)
        ax[1].set_title("Base position Y")
        ax[2].plot(x, self.dist_list)
        ax[2].set_title("Base distance from goal")
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