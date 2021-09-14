import numpy as np
import matplotlib.pyplot as plt

# make these smaller to increase the resolution
dx, dy = 0.01, 0.01

# generate 2 2d grids for the x & y bounds
x, y = np.mgrid[-0.78:1.78+dy:dy, -0.78:1.78+dx:dx]
print("x: ", x)
print("y: ", y)
print(type(x))
print(x.shape)

z = (1 - x/2 + x**5 + y**3) * np.exp(-x**2 - y**2)

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

z_min, z_max = -abs(z).max(), abs(z).max()

fig, ax = plt.subplots(1, 1)
center = np.array([1.2, 1.0])

ax.scatter(center[0], center[1], c="tab:blue", label="tab:blue", s=10., alpha=.5)
robot_circle = plt.Circle((1.2, 1.0), 0.01, color='g', fill=True)
ax.add_patch(robot_circle)

# ax = axs[0]
c = ax.pcolormesh(x, y, z, cmap='RdBu', vmin=z_min, vmax=z_max)
ax.set_title('heatmap')
fig.colorbar(c, ax=ax)

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

fig.tight_layout()
plt.show()