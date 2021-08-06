import numpy as np
import torch
import matplotlib.pyplot as plt
from icecream import ic

def visualize_vector_field(policy, device, min_max = [[-1,-1],[1,1]], fig_number=1):

    min_x = min_max[0][0]
    max_x = min_max[1][0]
    min_y = min_max[0][1]
    max_y = min_max[1][1]

    n_sample = 100
    x = np.linspace(min_x, max_x, n_sample)
    y = np.linspace(min_y, max_y, n_sample)

    xy = np.meshgrid(x, y)
    h = np.concatenate(xy[0])
    v = np.concatenate(xy[1])
    hv = torch.Tensor(np.stack([h, v]).T).float()
    if device is not None:
        hv = hv.to(device)

    vel = policy(hv)
    #vel = to_numpy(vel)
    vel = np.nan_to_num(vel)

    vel_x = np.reshape(vel[:, 0], (n_sample, n_sample))
    vel_y = np.reshape(vel[:, 1], (n_sample, n_sample))
    speed = np.sqrt(vel_x ** 2 + vel_y ** 2)
    speed = speed/np.max(speed)

    plt.streamplot(xy[0], xy[1], vel_x, vel_y, color=speed)

w = 5
Y, X = np.mgrid[-w:w:5j, -w:w:5j]
ic(Y)
ic(X)

import numpy as np
import matplotlib.pyplot as plt

# # Creating dataset
# x = np.arange(0, 10)
# y = np.arange(0, 10)
#
# # Creating grids
# X, Y = np.meshgrid(x, y)
# # ic(X)
# # ic(Y)
#
# # x-component to the right
# u = np.ones((15, 10))
#
# # y-component zero
# v = -np.ones((10, 10))
#
# fig = plt.figure(figsize=(12, 7))
#
# # Plotting stream plot
# plt.streamplot(X, Y, u, v, density=0.5)
#
# # show plot
# # plt.show()

import numpy as np
import matplotlib.pyplot as plt

# Creating data set
w = 3
Y, X = np.mgrid[-w:w:100j, -w:w:100j]
U1 = -1 - X ** 2 + Y
ic(type(U1))
ic(np.shape(U1))
V1 = 1 + X - Y ** 2
ic(np.shape(V1))

U2 = -1.1 - X ** 2 + Y
ic(np.shape(U1))
V2 = 2.1 + X - Y ** 2
# speed = np.sqrt(U ** 2 + V ** 2)

# Creating plot
fig = plt.figure(figsize=(12, 7))
plt.streamplot(X, Y, U1, V1, density=1)
plt.streamplot(X, Y, U2, V2, density=0.8)
# show plot
plt.show()