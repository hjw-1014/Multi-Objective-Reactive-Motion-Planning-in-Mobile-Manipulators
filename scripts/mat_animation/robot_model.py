"""

tiago+ robot model for CEP

"""

from math import sqrt, cos, sin, tan, pi

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from matplotlib import animation, rc
from IPython.display import HTML, Image

# Define robot configuration
radius = 0.27
robot_x = 0.
robot_y = 0.
des_x = 1.2
des_y = 1.0

class data_linewidth_plot():
    def __init__(self, x, y, **kwargs):
        self.ax = kwargs.pop("ax", plt.gca())
        self.fig = self.ax.get_figure()
        self.lw_data = kwargs.pop("linewidth", 1)
        self.lw = 1
        self.fig.canvas.draw()

        self.ppd = 72./self.fig.dpi
        self.trans = self.ax.transData.transform
        self.linehandle, = self.ax.plot([],[],**kwargs)
        if "label" in kwargs: kwargs.pop("label")
        self.line, = self.ax.plot(x, y, **kwargs)
        self.line.set_color(self.linehandle.get_color())
        self._resize()
        self.cid = self.fig.canvas.mpl_connect('draw_event', self._resize)

    def _resize(self, event=None):
        lw =  ((self.trans((1, self.lw_data))-self.trans((0, 0)))*self.ppd)[1]
        if lw != self.lw:
            self.line.set_linewidth(lw)
            self.lw = lw
            self._redraw_later()

    def _redraw_later(self):
        self.timer = self.fig.canvas.new_timer(interval=10)
        self.timer.single_shot = True
        self.timer.add_callback(lambda : self.fig.canvas.draw_idle())
        self.timer.start()

class Plotting:
    def __init__(self, robot_x_list=None, robot_y_list=None, dist_list=None, horizon=None):

        self.robot = None
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

                self.plot_robot(ax, robot_x_list[i], robot_y_list[i], radius)

                robot_circle = plt.Circle((robot_x_list[i], robot_y_list[i]), radius=0.01, color='y', fill=True)
                ax.text(robot_x_list[i], robot_y_list[i], s='cur', fontsize=8.)
                ax.add_patch(robot_circle)

                self.plot_des(ax, des_x, des_y, radius)

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


if __name__ == "__main__":

    fig, ax = plt.subplots()

    robot_x_list = []
    robot_y_list = []
    robot_x = 0.
    robot_y = 0.
    for i in range(100):
        x = robot_x + 0.005 * i
        y = robot_y + 0.005 * i
        robot_x_list.append(x)
        robot_y_list.append(y)

    plotting = Plotting(robot_x_list, robot_y_list)
    #plotting.save_gif(fig, ax)
    plotting.plot_animation()