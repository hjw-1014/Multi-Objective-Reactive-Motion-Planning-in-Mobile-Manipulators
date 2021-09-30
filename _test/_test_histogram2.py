import matplotlib.pyplot as plt
import numpy as np


success_list = [[1, 1, 0, 0, 1], [1, 1, 1, 0, 1]]
collision_list = [[0, 1, 0, 0, 1], [1, 0, 0, 0, 0]]
iter_list = [[5000, 5000, 4000, 3000, 2000], [1000, 7000, 4000, 2000, 6000]]
test_cep_models = ['track father', 'cascade control']
label = ['success', 'collision', 'iteration times']
x = np.arange(2)  # the label locations
width = 0.33  # the width of the bars
w = 0.165
height_suc = []
height_col = []
height_iter = []
for i in range(2):
    cur_suc = success_list[i]
    len_suc = float(len(cur_suc))
    suc = float(sum(cur_suc))
    height_suc.append(suc/len_suc)
    cur_col = collision_list[i]
    col = float(sum(cur_col))
    len_col = float(len(cur_col))
    height_col.append(col/len_col)
    cur_iter = iter_list[i]
    mean_iter = np.mean(cur_iter)
    height_iter.append(mean_iter/10000.)


fig, ax = plt.subplots()
rects1 = ax.bar(x - width/2, height_suc, w, label=label[0])
rects2 = ax.bar(x, height_col, w, label=label[1])
rects3 = ax.bar(x + width/2, height_iter, w, label=label[2])

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Scores')
ax.set_title('Scores by group and gender')
ax.set_xticks(x)
ax.set_xticklabels(test_cep_models)
ax.legend()

ax.bar_label(rects1, padding=3)
ax.bar_label(rects2, padding=3)
ax.bar_label(rects3, padding=3)
fig.tight_layout()

plt.show()
