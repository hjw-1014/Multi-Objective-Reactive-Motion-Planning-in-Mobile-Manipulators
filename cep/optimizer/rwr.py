import torch
from .w_mle import weighted_mle

#import matplotlib.pyplot as plt
#import math
import torch

class RWR():

    def __init__(self, beta):
        self.beta = beta
        self.best_solution = BestSolution()

    def optimize(self, theta, J):
        indices = torch.argmax(J)
        J_max = J[indices].clone()

        self.best_solution.set_best(theta[indices, :], J_max)

        J -= J_max
        d = torch.exp(self.beta * J)
        mu, std = weighted_mle(theta, d)  # Maximum Likelihood estimation

        # plt.clf()
        # plt.plot(theta, d, '*')
        # plt.draw()
        # plt.pause(0.001)
        # plt.show()
        # if math.isnan(mu[0]):
        #     print(mu)
        #
        # print('mean:',mu, 'and variance:', std)
        return mu, std

    def init_optimization(self):
        self.best_solution.clear()

class BestSolution():

    def __init__(self):
        self.x_optima = None
        self.J_optima = -10000000000000000000.

    def set_best(self, x, J):
        if J > self.J_optima or self.x_optima is None:
            self.J_optima = J
            self.x_optima = x

    def clear(self):
        self.x_optima = None
        self.J_optima = -10000000000000000000.




