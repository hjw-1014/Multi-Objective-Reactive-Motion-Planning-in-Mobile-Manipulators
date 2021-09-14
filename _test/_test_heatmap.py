"""
    Draw a thermal image
"""
import numpy as np
import matplotlib.pyplot as plt

# Prepare data
num = 1000
x, y = np.meshgrid(np.linspace(-7.8, 17.8, 256),
                   np.linspace(-7.8, 17.8, 256))

print("x: ", x)
print("y: ", y)
print(type(x))
print(x.shape)
#z = (1 - x / 2 + x ** 5 + y ** 3) * np.exp(-x ** 2 - y ** 2)
z = x + y
print(z[-1][-1])
# Draw pictures
plt.figure("imshow", figsize=(8, 6), facecolor="lightgray")
plt.title("imshow", fontsize=18)

plt.imshow(z, cmap="jet", origin="lower")
plt.colorbar()
plt.show()

