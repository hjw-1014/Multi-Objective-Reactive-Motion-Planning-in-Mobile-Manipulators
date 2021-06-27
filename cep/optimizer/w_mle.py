import torch


def weighted_mle(theta, weights):
    theta_dim = theta.shape[1]
    n_particles = theta.shape[0]

    sumD = torch.sum(weights)
    sumD2 = torch.sum(weights ** 2)
    Z = sumD - sumD2 / sumD


    _mu = torch.bmm(weights.repeat(theta_dim,1).view(theta_dim,1, n_particles), theta.T[:,:,None]).view(theta_dim) / sumD

    delta2 = (theta - _mu) ** 2
    _std = torch.sqrt(torch.bmm(weights.repeat(theta_dim,1).view(theta_dim,1, n_particles), delta2.T[:,:,None]).view(theta_dim) / Z)
    return _mu, _std
