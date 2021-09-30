import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

# # example data
# mu = 100  # mean of distribution
# sigma = 15  # standard deviation of distribution
# x = mu + sigma * np.random.randn(437)
#
# num_bins = 50
#
# fig, axs = plt.subplots(1, 3)
#
# # the histogram of the data
# for i in range(3):
#     ax = axs[i]
#     n, bins, patches = ax.hist(x, num_bins, density=True)
#
#     # add a 'best fit' line
#     # y = ((1 / (np.sqrt(2 * np.pi) * sigma)) *
#     #      np.exp(-0.5 * (1 / sigma * (bins - mu))**2))
#     # ax.plot(bins, y, '--')
#     ax.set_xlabel('Smarts')
#     ax.set_ylabel('Probability density')
#     ax.set_title(r'Histogram of IQ: $\mu=100$, $\sigma=15$')
#
# # Tweak spacing to prevent clipping of ylabel
# fig.tight_layout()
# plt.show()
success_list = [[1, 1, 0, 0, 1], [1, 1, 1, 0, 1]]
collision_list = [[0, 1, 0, 0, 1], [1, 0, 0, 0, 0]]
test_cep_models = ['track father', 'cascade control']
num = len(success_list)  # Number of policies

fig, axs = plt.subplots(1, num)

# the histogram of the data
for i in range(num):
    cur_suc = success_list[i]
    len_suc = float(len(cur_suc))
    suc = float(sum(cur_suc))
    cur_col = collision_list[i]
    col = float(sum(cur_col))
    len_col = float(len(cur_col))
    height = [suc/len_suc, col/len_col]

    ax = axs[i]
    label = ['success', 'collision']
    colors = ['tan', 'lime']
    x = np.arange(2)
    for i in range(len(label)):
        ax.bar(x[i], height=height[i], color=colors[i], label=label[i])
        ax.bar_label(height[i])
    ax.legend(prop={'size': 10})
    ax.set_ylim(0., 1.)
    ax.set_xlabel('success | collision')
    ax.set_ylabel('Success rate and collision rate')
    ax.set_title("{}".format(test_cep_models[i]))

# Tweak spacing to prevent clipping of ylabel
fig.tight_layout()
plt.show()