import numpy as np
import matplotlib.pyplot as plt
import torch.distributions as tdist
import torch, math

def gen_gridmap():
    """
        Return a grid map with tensorsize([nc*nr, 2]) for heatmap generation
    """

    dx, dy = -0.01, 0.01

    # generate 2 2d grids for the x & y bounds
    y, x = np.mgrid[1.78:-0.78 + dx:dx, -0.78:1.78 + dy:dy]

    nr = len(x)
    nc = len(y)
    grid_map = []
    cur_grid = [0, 0]
    for i in range(nr):
        for j in range(nc):
            cur_grid[0] = x[i][j]
            cur_grid[1] = y[i][j]
            grid_map.append(cur_grid)
            cur_grid = [0, 0]

    grid_map = torch.tensor(grid_map)

    return grid_map

def gen_heatmap(log_map, closest_point, current_point):
    """
        log_map: torch.tensorsize([nc* nr, 2])
    """
    fig, ax = plt.subplots(1, 1)  # TODO: Initialize fig
    dx, dy = -0.01, 0.01

    # generate 2 2d grids for the x & y bounds
    y, x = np.mgrid[1.78:-0.78 + dx:dx, -0.78:1.78 + dy:dy]

    nr = len(x)
    nc = len(y)

    log_map = log_map.reshape((nr, nc))

    log_max, log_min = log_map.max(), log_map.min()

    # Mark the goal point
    # goal_circle = plt.Circle((1.2, 1.0), 0.01, color='r', fill=True)
    # ax.text(1.2, 1.0, s='goal', fontsize=8.)
    # ax.add_patch(goal_circle)

    # Mark the current point
    current_circle = plt.Circle((current_point[0], current_point[1]), 0.01, color='y', fill=True)
    ax.text(current_point[0], current_point[1], s='Current', fontsize=14.)
    ax.add_patch(current_circle)

    # Mark the closest point
    for i in range(len(closest_point)):
        closest_circle = plt.Circle((closest_point[i][0], closest_point[i][1]), 0.01, color='g', fill=True)
        ax.text(closest_point[i][0], closest_point[i][1], s='Closest', fontsize=14.)
        ax.add_patch(closest_circle)

    # Mark the arrow
    for i in range(len(closest_point)):
        delta_x, delta_y = closest_point[i][0] - current_point[0], closest_point[i][1] - current_point[1]
        delta = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        ax.arrow(current_point[0], current_point[1], dx=delta_x / delta * 0.06, dy=delta_y / delta * 0.05, width=0.0005,
                 head_width=0.03, color='y')

    c = ax.pcolor(x, y, log_map, cmap='RdBu', vmin=log_min, vmax=log_max)
    ax.set_title('Closest point heatmap')

    fig.colorbar(c, ax=ax)

    fig.tight_layout()
    plt.show()

    return fig

# # make these smaller to increase the resolution
# dx, dy = 0.01, 0.01
#
#
# # generate 2 2d grids for the x & y bounds
# x, y = np.mgrid[-0.78:1.78+dy:dy, -0.78:1.78+dx:dx]
# print("x: ", x)
# print("y: ", y)
# print(type(x))
# print(x.shape)
#
# z = (1 - x/2 + x**5 + y**3) * np.exp(-x**2 - y**2)

# x and y are bounds, so z should be the value *inside* those bounds.
# Therefore, remove the last value from the z array.
# z = z[:-1, :-1]
# print(z)
# print(z[0][-1])
# z = np.ones((x.shape))
# z[78][78] = 5.

# for i in range(200, len(x)):
#     for j in range(200, len(x)):
#         z[i][j] = 0.5

# z_min, z_max = -abs(z).max(), abs(z).max()
#
# fig, ax = plt.subplots(1, 1)
# center = np.array([1.2, 1.0])
#
# ax.scatter(center[0], center[1], c="tab:blue", label="tab:blue", s=10., alpha=.5)
# robot_circle = plt.Circle((1.2, 1.0), 0.01, color='g', fill=True)
# ax.add_patch(robot_circle)
#
# # ax = axs[0]
# c = ax.pcolormesh(x, y, z, cmap='RdBu', vmin=z_min, vmax=z_max)
# ax.set_title('heatmap')
# fig.colorbar(c, ax=ax)

# ax = axs[0, 1]
# c = ax.pcolormesh(x, y, z)
# ax.set_title('pcolormesh')
# fig.colorbar(c, ax=ax)

# ax = axs[1, 0]
# c = ax.imshow(z, cmap='RdBu', vmin=z_min, vmax=z_max,
#               extent=[x.min(), x.max(), y.min(), y.max()],
#               interpolation='nearest', origin='lower', aspect='auto')
# ax.set_title('image (nearest, aspect="auto")')
# fig.colorbar(c, ax=ax)
#
# ax = axs[1, 1]
# c = ax.pcolorfast(x, y, z, cmap='RdBu', vmin=z_min, vmax=z_max)
# ax.set_title('pcolorfast')
# fig.colorbar(c, ax=ax)

pos_n = torch.tensor([[-0.3, -0.3], [1.5, 1.5]])
pos_n = torch.tensor([[1.5, 1.5]])
pos_n = torch.tensor([[-0.3, -0.3]])
p_dx = []
var = torch.eye(2).float() * 1.

for i in range(len(pos_n)):
    cur_gaussian = tdist.MultivariateNormal(pos_n[i], var)
    p_dx.append(cur_gaussian)

gg = []
closest_point = pos_n
current_point = [-0.15, 1.1]
for i in range(len(pos_n)):
    grid_map = gen_gridmap()
    gg.append(torch.unsqueeze(p_dx[i].log_prob(grid_map), dim=1))
log_map = torch.exp(torch.logsumexp(torch.stack(gg, dim=2), dim=2)).reshape(len(grid_map), )
fig = gen_heatmap(log_map=log_map, closest_point=closest_point, current_point=current_point)

fig.tight_layout()
plt.show()