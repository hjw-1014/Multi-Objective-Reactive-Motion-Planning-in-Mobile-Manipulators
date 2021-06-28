import torch
import torch.autograd as autograd
import numpy as np

from cep.utils import torch2numpy, numpy2torch

import torch.nn as nn

from cep.optimizer import RWR

from cep.maps import Map

import time

# from torch.multiprocessing import Pool
# torch.multiprocessing.set_start_method('fork', force=True)


# todo: Use Sympy for systems evolution
def solve_euler(q, dq, dt):
    return q + dq * dt

class Multi_EBMControl():
    def __init__(self, energy_tree, device, dim=7, dt=0.005, optimization_steps=10, n_particles=1000, var_0=10., stochastic=False):

        self.device = device
        self.energy_tree = energy_tree
        self.dim = dim

        ## Controller ##
        self.stochastic = stochastic

        ### Optimization ##
        self.optimization_steps = optimization_steps
        self.n_particles = n_particles
        self.var_0 = var_0
        self.optimizer = RWR(beta=0.1)

        self.log_p_dq = torch.zeros((n_particles)) # TODO: 06.28

        ## Position Action ##
        self.dt = dt

    def policy(self, state):
        ## 1. State to Torch ##
        state_t = numpy2torch(state, self.device)
        ## 2.Compute Action ##
        action_t = self.get_action(state_t, stochastic=self.stochastic)  # torch.Size([7])
        ## 3. action2numpy
        if action_t is None:
            print('block')
        action = torch2numpy(action_t)
        return action

    def get_action(self, state, stochastic='False'):
        ## 1. Conditioning ##
        t0 = time.time()

        # TODO: initialize/update Multivaritae Gaussian distribution , self.p_dx = tdist.MultivariateNormal(mu, self.var) in TaskgotoLeaf
        #self.energy_tree.set_context(state)

        for jj in range(len(self.energy_tree)): # TODO: 06.28
            self.energy_tree[jj].set_context(state)

        ## 2. Compute optimal Action ##
        t1 = time.time()
        a_opt = self.compute_optimal_action()

        t2 = time.time()
        #print('inter1:  {}, inter 2: {}'.format(t1-t0, t2-t1))
        return a_opt

    def compute_optimal_action(self):
        mu = torch.zeros(self.dim).to(self.device)
        std = torch.eye(self.dim).to(self.device)*self.var_0

        self.optimizer.init_optimization()
        white_noise = torch.randn(self.n_particles, self.dim).to(self.device)
        for i in range(self.optimization_steps):
            t0 = time.time()
            action = torch.matmul(std, white_noise.T).T + mu  # Random sample, torch.Size([1000, 7])

            t1 = time.time()

            for ii in range(len(self.energy_tree)): # TODO:
                self.log_p_dq += self.energy_tree[ii].log_prob(action)

            #log_p_dq = self.energy_tree.log_prob(action)  # Energy, torch.Size([1000]) # TODO: 06.28

            t2 = time.time()
            mu, var = self.optimizer.optimize(action, self.log_p_dq)  # Update mu and var based on sampled action and energy
            # mu, var -> torch.Size([7])

            t3 = time.time()
            #print('int 1: {}, int 2: {}, int3: {}'.format(t1-t0,t2-t1,t3-t2))

            std = torch.diag(torch.sqrt(var))  # torch.Size([7, 7])

        return self.optimizer.best_solution.x_optima  # torch.Size([7])

class EBMControl():
    def __init__(self, energy_tree, device, dim=7, dt=0.005, optimization_steps=10, n_particles=1000, var_0=10., stochastic=False):

        self.device = device
        self.energy_tree = energy_tree
        self.dim = dim

        ## Controller ##
        self.stochastic = stochastic

        ### Optimization ##
        self.optimization_steps = optimization_steps
        self.n_particles = n_particles
        self.var_0 = var_0
        self.optimizer = RWR(beta=0.1)

        ## Position Action ##
        self.dt = dt

    def policy(self, state):
        ## 1. State to Torch ##
        state_t = numpy2torch(state, self.device)
        ## 2.Compute Action ##
        action_t = self.get_action(state_t, stochastic=self.stochastic)  # torch.Size([7])
        ## 3. action2numpy
        if action_t is None:
            print('block')
        action = torch2numpy(action_t)
        return action

    def get_action(self, state, stochastic='False'):
        ## 1. Conditioning ##
        t0 = time.time()

        # TODO: initialize/update Multivaritae Gaussian distribution , self.p_dx = tdist.MultivariateNormal(mu, self.var) in TaskgotoLeaf
        self.energy_tree.set_context(state)

        ## 2. Compute optimal Action ##
        t1 = time.time()
        a_opt = self.compute_optimal_action()

        t2 = time.time()
        #print('inter1:  {}, inter 2: {}'.format(t1-t0, t2-t1))
        return a_opt

    def compute_optimal_action(self):
        mu = torch.zeros(self.dim).to(self.device)
        std = torch.eye(self.dim).to(self.device)*self.var_0

        self.optimizer.init_optimization()
        white_noise = torch.randn(self.n_particles, self.dim).to(self.device)
        for i in range(self.optimization_steps):
            t0 = time.time()
            action = torch.matmul(std, white_noise.T).T + mu  # Random sample, torch.Size([1000, 7])

            t1 = time.time()

            log_p_dq = self.energy_tree.log_prob(action)  # Energy, torch.Size([1000]) # TODO:

            t2 = time.time()
            mu, var = self.optimizer.optimize(action, log_p_dq)  # Update mu and var based on sampled action and energy
            # mu, var -> torch.Size([7])

            t3 = time.time()
            #print('int 1: {}, int 2: {}, int3: {}'.format(t1-t0,t2-t1,t3-t2))

            std = torch.diag(torch.sqrt(var))  # torch.Size([7, 7])

        return self.optimizer.best_solution.x_optima  # torch.Size([7])


class EnergyTree(nn.Module):
    '''
    An Energy Tree is the base node of a Tree. It will be composed of a Mapping, that transforms state and action to some latent state
    and branches

    '''
    def __init__(self, branches, map=None, i_temperatures=None):
        super(EnergyTree, self).__init__()
        if map is None:
            self.map = Map()
        else:
            self.map = map

        if i_temperatures is None:
            i_temperatures = torch.ones(len(branches))
        self.i_temperatures = nn.Parameter(i_temperatures)
        self.branches = nn.ModuleList(branches)

    def set_context(self, state):

        state_z = self.map.map_state(state)

        #processes = []
        for branch in self.branches:
            self.set_context_i(branch, state_z)
        #     branch.share_memory()
        #     p = mp.Process(target=self.set_context_i, args=(branch, state_z,))
        #     p.start()
        #     processes.append(p)
        # for p in processes:
        #     p.join()
        #time.sleep(0.1)

    def set_context_i(self, energy, state):
        energy.set_context(state)

    def log_prob(self, action):  # action -> torch.Size([1000, 7])
        action_z = self.map.map_action(action)   # FK_map, action_z -> torch.Size([1000, 7, 6]) | # Selection_map, action_z -> torch.Size([1000, 6])
        logp_a = torch.zeros(action.shape[0]).to(action)
        for idx, branch in enumerate(self.branches):
            logp_a += self.i_temperatures[idx] * branch.log_prob(action_z)
        return logp_a  # torch.Size([1000])

        # pool = Pool(processes=len(self.branches))
        # idx = 0
        # with Pool(processes=len(self.branches)) as p:
        #     log_prob = p.map(self.log_prob_i, self.branches,)


    def log_prob_i(self, energy, action):
        #print(ind)
        return energy.log_prob(action)






