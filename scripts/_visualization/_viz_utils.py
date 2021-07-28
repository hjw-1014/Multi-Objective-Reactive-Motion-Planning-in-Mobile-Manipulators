import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def visualize_vector_field(self, min_x=-1.5, min_y=-1.5, max_x=1.5, max_y=1.5):  # TODO: add on 07.21, change on 07.24
    '''
        Plot vector field based on groud truth path which is generated by RRTstar using MoveIt
    '''

    traj = self.open_json('qtrjs.json')  # Get 2d trajectory generated by MoveIt
    xy_traj = self.get_x_y_traj(traj)

    fig, ax = plt.subplots()  # TODO: Initialize fig

    n_sample = 301
    x = np.linspace(min_x, max_x, n_sample)
    y = np.linspace(min_y, max_y, n_sample)
    XY = np.meshgrid(x, y)  # TODO: So each step is 0.01,  XY is a list

    # Get the velocity map with dimension np.array((301x301, 2))
    uv = self.cascade_control_all_points()  # TODO: Add repulsive potential field

    vel_x = np.reshape(uv[:, 0], (n_sample, n_sample))
    vel_y = np.reshape(uv[:, 1], (n_sample, n_sample))

    speed = (vel_x ** 2 + vel_y ** 2) ** (1 / 2)
    lw = 5 * speed / speed.max()
    # mask = np.zeros(vel_x.shape, dtype=bool)
    # mask[85:115, 185:215] = True
    # vel_x = np.ma.array(vel_x, mask=mask) # TODO: Try to add a mask for the object. on 07.24

    ax.streamplot(XY[0], XY[1], vel_x, vel_y,
                  density=[0.5, 1.],
                  linewidth=1.,
                  color=speed,
                  )  # TODO: Draw the vector field

    xy_traj = self.xy_traj
    x_traj = [0] * self.num_path_points
    y_traj = [0] * self.num_path_points
    for j in range(self.num_path_points):
        x_traj[j] = xy_traj[j][0]
        y_traj[j] = xy_traj[j][1]

    ax.plot(x_traj, y_traj, 'C3', lw=1.)  # TODO: Draw the groud truth path generated by MoveIt

    left, bottom, width, height = (0.35, 0.35, 0.3, 0.3)
    ax.add_patch(Rectangle((left, bottom), width, height, facecolor="black", alpha=0.5))  # TODO: Draw the box

    ax.set_title('Vector field')
    ax.grid(True)
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    plt.show()