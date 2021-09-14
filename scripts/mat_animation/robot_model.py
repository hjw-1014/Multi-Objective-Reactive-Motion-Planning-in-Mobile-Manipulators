"""

tiago+ robot model for CEP

"""

from math import sqrt, cos, sin, tan, pi

import matplotlib.pyplot as plt
import numpy as np

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

def plot_robot(ax, robot_x, robot_y, radius):

    robot_color = 'blue'
    robot_circle = plt.Circle((robot_x, robot_y), radius, color=robot_color,  fill=False)
    ax.add_patch(robot_circle)

def plot_des(ax, des_x, des_y, radius):

    desire_cicle = plt.Circle((des_x, des_y), radius, color='g', fill=False)
    ax.add_patch(desire_cicle)

fig, ax = plt.subplots(figsize=(9, 9))

plot_des(ax, des_x, des_y, radius)

robot_x_list = []
robot_y_list = []

for i in range(100):
    x = robot_x + 0.005 * i
    y = robot_y + 0.005 * i
    robot_x_list.append(x)
    robot_y_list.append(y)

for i in range(100):
    plt.cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])
    ax.set_xlim(-1, 2)
    ax.set_ylim(-1, 2)
    plot_robot(ax, robot_x_list[i], robot_y_list[i], radius)
    plt.pause(0.000001)

l = data_linewidth_plot([0., 0.2], [0., 0.2], ax=ax, label='some 1 data unit wide line',
                        linewidth=0.01, alpha=1.)
#plot_robot(ax, robot_x, robot_y, radius)
plt.show()