import torch
import torch.distributions as tdist
import time


def fit_with_wMLE(x, r):
    r = r - torch.min(r)
    normalization = torch.sum(r)

    wx = r.repeat(2,1).T * x
    mu = torch.sum(wx)/normalization
    print(mu)




mu1 = torch.randn(2) +2
var1 = torch.eye(2)
p1 = tdist.MultivariateNormal(loc=mu1, covariance_matrix=var1)

mu2 = torch.randn(2) - 2
var2 = torch.eye(2)
p2 = tdist.MultivariateNormal(loc=mu2, covariance_matrix=var2)

log_p = lambda x: torch.logsumexp(torch.cat([p1.log_prob(x).view(-1,1), p2.log_prob(x).view(-1,1)],1),1)


## Log P(x)

x = torch.rand(1000,2)*10. - 5.
log_px = log_p(x)

fit_with_wMLE(x, log_px)


#print(log_px)










