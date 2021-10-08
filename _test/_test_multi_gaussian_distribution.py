# Test multi gaussian distribution
import torch
import torch.distributions as td
import matplotlib.pyplot as plt
from icecream import ic

g1 = td.Normal(torch.tensor([0.0]), torch.tensor([1.]))
g2 = td.Normal(torch.tensor([5.0]), torch.tensor([1.]))
ls = torch.linspace(-10, 10, 201)
print(ls)

g1_t = g1.log_prob(ls)
g2_t = g2.log_prob(ls)
g12_t = torch.exp(g1_t + g2_t)

#g12_s = torch.exp(g1_t) + torch.exp(g2_t)
g1_u = torch.unsqueeze(g1_t, dim=1)
g2_u = torch.unsqueeze(g2_t, dim=1)
g12_s = torch.exp(torch.logsumexp(torch.stack([g1_u, g2_u], dim=2), dim=2))
print(g12_t)

figure, ax = plt.subplots(1, 4, figsize=(12, 3))
# for i in range(4):
#     ax[i].set_ylim(0, 3)
ax[0].plot(ls, g1_t)
ax[0].set_title("log_prob(p1)")
ax[1].plot(ls, g2_t)
ax[1].set_title("log_prob(p2)")
ax[1].set_xlim(-10, 10)
ax[2].plot(ls, g12_t)
ax[2].set_title("exp(sum log_i(x))")
ax[3].plot(ls, g12_s)
ax[3].set_title("exp(logsumexp(x))")
plt.show()
