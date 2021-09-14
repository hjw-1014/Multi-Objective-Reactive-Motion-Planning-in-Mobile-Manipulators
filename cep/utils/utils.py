import numpy as np
import torch
import matplotlib.pyplot as plt
from icecream import ic

def numpy2torch(x, device='cpu'):
    return torch.from_numpy(x).float().to(device)

def torch2numpy(x):
    if x is None:
        print(x)

    if x.device.type=='cuda':
        return x.cpu().detach().numpy()
    else:
        return x.detach().numpy()

def qten2qnine(q):
    return q

def gen_gridmap():

    """
        Return a grid map with tensorsize([nc*nr, 2]) for heatmap generation
    """

    dx, dy = -0.01, 0.01

    # generate 2 2d grids for the x & y bounds
    y, x = np.mgrid[1.78:-0.78 + dx:dx, -0.78:1.78 + dy:dy]
    ic(x)
    ic(y)

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

def gen_heatmap(log_map):

    """
        log_map: torch.tensorsize([nc* nr, 2])
    """
    fig, axs = plt.subplots(1, 2)  # TODO: Initialize fig
    dx, dy = -0.01, 0.01

    # generate 2 2d grids for the x & y bounds
    y, x = np.mgrid[1.78:-0.78 + dx:dx, -0.78:1.78 + dy:dy]

    nr = len(x)
    nc = len(y)

    log_map = log_map.reshape((nr, nc))

    log_max, log_min = log_map.max(), log_map.min()

    ax = axs[0]
    c = ax.pcolor(x, y, log_map, cmap='RdBu', vmin=log_min, vmax=log_max)
    ax.set_title('pcolor')
    fig.colorbar(c, ax=ax)

    ax = axs[1]
    c = ax.pcolormesh(x, y, log_map)
    ax.set_title('pcolormesh')
    fig.colorbar(c, ax=ax)

    fig.tight_layout()
    plt.show()
