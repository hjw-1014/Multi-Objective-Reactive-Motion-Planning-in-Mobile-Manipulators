import torch

import numpy as np
import matplotlib.pyplot as plt


class CMA_ES():

    def __init__(self, dim, mean, std, population_size = 1000):

        self.counteval = 0
        N = dim

        self.pc = N * [0]  # evolution path for C
        self.ps = N * [0]  # and for sigma

        self.xmean = mean
        self.C = torch.eye(N)
        self.sigma = std

        self.best_solution = BestSolution()

        self.params = CMAESParameters(N, population_size)

    def get_mean_and_var(self):
        mean = self.xmean
        var = self.sigma**2*self.C
        return mean, var


    def optimize(self, theta, J):
        N = theta.shape[1]
        n_particles = theta.shape[0]

        self.counteval += len(J)
        par = self.params
        xold = self.xmean.clone()  # not a copy, xmean is assigned anew later


        # Sort Particles by Quality
        J_sorted, indices = torch.sort(J, 0)
        theta_sorted = theta[indices.view(-1),:]

        # Compute new mean
        J_sortedn = J_sorted  - J_sorted[-1]
        d = torch.exp(J_sortedn)
        sumD = torch.sum(d)
        self.xmean = torch.bmm(d.repeat(1, N).view(N, 1, n_particles),
                        theta_sorted.view(N, n_particles, 1)).view(N) / sumD


        plt.clf()
        plt.plot(theta_sorted, d, '*')
        plt.draw()
        plt.pause(0.001)

        self.best_solution.set_best(theta_sorted[-1], J_sorted[-1])

        theta_sorted = theta_sorted.flip(0)

        # Cumulation: update evolution paths
        y = self.xmean - xold
        z = torch.matmul(torch.rsqrt(self.C), y)
        csn = (par.cs * (2 - par.cs) * par.mueff)**0.5 / self.sigma
        for i in range(N):  # update evolution path ps
            self.ps[i] = (1 - par.cs) * self.ps[i] + csn * z[i]


        ccn = (par.cc * (2 - par.cc) * par.mueff)**0.5 / self.sigma
        # turn off rank-one accumulation when sigma increases quickly
        hsig = (sum(x**2 for x in self.ps) / N  # ||ps||^2 / N is 1 in expectation
                / (1-(1-par.cs)**(2*self.counteval/par.lam))  # account for initial value of ps
                < 2 + 4./(N+1))  # should be smaller than 2 + ...
        for i in range(N):  # update evolution path pc
            self.pc[i] = (1 - par.cc) * self.pc[i] + ccn * hsig * y[i]

        ### Adapt covariance matrix C
        # minor adjustment for the variance loss from hsig
        c1a = par.c1 * (1 - (1 - hsig ** 2) * par.cc * (2 - par.cc))
        self.C = self.C * (1 - c1a - par.cmu * sum(par.weights))  # C *= 1 - c1 - cmu * sum(w)

        self.C = addouter(self.C, self.pc, par.c1) # C += c1 * pc * pc^T, so-called rank-one update

        for k, wk in enumerate(par.weights):  # so-called rank-mu update
            diff_xk = theta_sorted[k, :] - xold
            if wk < 0:  # guaranty positive definiteness
                mahalanobis_norm = torch.sum(xi**2 for xi in torch.matmul(torch.rsqrt(self.C), diff_xk))**0.5

                wk *= N * (self.sigma / mahalanobis_norm) ** 2
            self.C = addouter(self.C, diff_xk, wk * par.cmu / self.sigma ** 2)  # C += c1 * pc * pc^T, so-called rank-one update

        ### Adapt step-size sigma
        cn, sum_square_ps = par.cs / par.damps, sum(x ** 2 for x in self.ps)
        log_sigma_mult = min(1., cn * (sum_square_ps / N - 1) / 2)
        self.sigma *= np.exp(log_sigma_mult)


class CMAESParameters():
    """static "internal" parameter setting for `CMAES`

    """
    default_popsize = '4 + int(3 * log(N))'
    def __init__(self, N, popsize=None,
                 RecombinationWeights=None):
        """set static, fixed "strategy" parameters once and for all.

        Input parameter ``RecombinationWeights`` may be set to the class
        `RecombinationWeights`.
        """
        self.dimension = N
        self.chiN = N**0.5 * (1 - 1. / (4 * N) + 1. / (21 * N**2))

        # Strategy parameter setting: Selection
        self.lam = eval(safe_str(popsize if popsize else
                                 CMAESParameters.default_popsize,
                                 {'int': 'int', 'log': 'log', 'N': N}))
        self.mu = int(self.lam / 2)  # number of parents/points/solutions for recombination
        if RecombinationWeights:
            self.weights = RecombinationWeights(self.lam)
            self.mueff = self.weights.mueff
        else:  # set non-negative recombination weights "manually"
            _weights = [np.log(self.lam / 2 + 0.5) - np.log(i + 1) if i < self.mu else 0
                        for i in range(self.lam)]
            w_sum = sum(_weights[:self.mu])
            self.weights = [w / w_sum for w in _weights]  # sum is one now
            self.mueff = sum(self.weights[:self.mu])**2 / \
                         sum(w**2 for w in self.weights[:self.mu])  # variance-effectiveness of sum w_i x_i

        # Strategy parameter setting: Adaptation
        self.cc = (4 + self.mueff/N) / (N+4 + 2 * self.mueff/N)  # time constant for cumulation for C
        self.cs = (self.mueff + 2) / (N + self.mueff + 5)  # time constant for cumulation for sigma control
        self.c1 = 2 / ((N + 1.3)**2 + self.mueff)  # learning rate for rank-one update of C
        self.cmu = min([1 - self.c1, 2 * (self.mueff - 2 + 1/self.mueff) / ((N + 2)**2 + self.mueff)])  # and for rank-mu update
        self.damps = 2 * self.mueff/self.lam + 0.3 + self.cs  # damping for sigma, usually close to 1

        if RecombinationWeights:
            self.weights.finalize_negative_weights(N, self.c1, self.cmu)
        # gap to postpone eigendecomposition to achieve O(N**2) per eval
        # 0.5 is chosen such that eig takes 2 times the time of tell in >=20-D
        self.lazy_gap_evals = 0.5 * N * self.lam * (self.c1 + self.cmu)**-1 / N**2


class BestSolution():
    def __init__(self):
        self.J = -np.inf
        self.x = 0.

    def set_best(self, x, J):
        pass

def safe_str(s, known_words=None):
    """return ``s`` as `str` safe to `eval` or raise an exception.
    Strings in the `dict` `known_words` are replaced by their values
    surrounded with a space, which the caller considers safe to evaluate
    with `eval` afterwards.

    """
    safe_chars = ' 0123456789.,+-*()[]e'
    if s != str(s):
        return str(s)
    if not known_words:
        known_words = {}
    stest = s[:]  # test this string
    sret = s[:]  # return this string
    for word in sorted(known_words.keys(), key=len, reverse=True):
        stest = stest.replace(word, '  ')
        sret = sret.replace(word, " %s " % known_words[word])
    for c in stest:
        if c not in safe_chars:
            raise ValueError('"%s" is not a safe string'
                             ' (known words are %s)' % (s, str(known_words)))
    return sret


def addouter(C, b, factor=1):
    """Add in place `factor` times outer product of vector `b`,

    without any dimensional consistency checks.
    """
    for i in range(C.shape[0]):
        for j in range(C.shape[1]):
            C[i,j] += factor * b[i] * b[j]
    return C