import os

import torch
import torch.distributions as tdist
import numpy as np
import cascade_control_dx
from .energy_leaf import EnergyLeaf
from .energy_leaf import EnergyLeaf_x
#from _utils import torch2numpy
from cep.utils import eul2rot, rot2eul, rot2quat
from cep.liegroups.torch import SO3, SE3
from icecream import ic
import matplotlib.pyplot as plt
import time

global num
num = 1

global closest_points
closest_points = []

global count
count = 1

global CREATE_DIR
CREATE_DIR = False

global path
path = ""

def torch2numpy(x):
    if x is None:
        print(x)

    if x.device.type == 'cuda':
        return x.cpu().detach().numpy()
    else:
        return x.detach().numpy()

class TaskGoToLeaf(EnergyLeaf):
    def __init__(self, dim=6, A = None, b = None, R = None, var=None):
        super(TaskGoToLeaf, self).__init__()
        self.dim = dim

        if A is None:
            A = torch.eye(self.dim).float()
        self.register_buffer('A', A)
        if b is None:
            b = torch.zeros(3).float()
        self.register_buffer('b', b)
        if var is None:
            var = torch.eye(self.dim).float() * 1.
        self.register_buffer('var', var)

        if R is None:
            R = torch.eye(3).float()

        R_inv = torch.inverse(R)
        self.register_buffer('R', R)
        self.register_buffer('R_inv', R_inv)

        ## variables for computation ##
        self.p_dx = None

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''
        x = state[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
        v = state[1]  # Tensor (1, 6), end-effector spatial velocity V_b
        #print('state:', state)
        # print('x: ', x)
        # print('v: ', v)

        #print('self.R_inv: ', self.R_inv) # Tensor (4, 4)
        #print('R: ', self.R) # Tensor (4, 4)

        Htl = torch.matmul(self.R_inv, x) # R_inv * X
        #print('Htl: ', Htl)
        Xe = SE3.from_matrix(Htl) # <cep.liegroups.torch.se3.SE3Matrix>, SE(3)
        #print('Xe: ', Xe)
        xtl = Xe.log() # Tensor(1, 6), (omega, V)
        #print('xtl: ', xtl)
        vtl = -xtl
        A = SE3.from_matrix(self.R) # <cep.liegroups.torch.se3.SE3Matrix>, SE(3), R
        #print('A: ', A)
        Adj_lw = A.adjoint() # Adjoint map (Spatial velocity from one frame to another frame), Tensor (6,6),
        #print('Adj_lw: ', Adj_lw)
        ve_w = torch.matmul(Adj_lw, vtl) # Tensor(6, 1)
        #print('v_ew: ', ve_w)
        ###########################################

        scale = 20.
        mu = scale * ve_w - 1.2 * scale * v
        #print('mu: ', mu)  # Tensor(6, 1)

        self.p_dx = tdist.MultivariateNormal(mu, self.var)  # self.var->torch.size(6, 6)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        # TODO:
        action = action[:, :self.dim]  # torch.Size([1000, 6])
        return self.p_dx.log_prob(action)  # torch.Size([1000])

class JointGoToLeaf(EnergyLeaf):
    def __init__(self, dim=7, Kp = 1., Kv = 1.,
                 q_des=torch.tensor([2.0, 0.1, 2.0, 1.0, 0.2, -1.0, 0.3]),
                 # ([2.215, 0.088, 2.156, 1.060, 0.238, -1.023, 0.373]),
                 dq_des=torch.tensor([0., 0., 0., 0., 0., 0., 0.]),
                 var=torch.eye(7).float() * 100.):

        super(JointGoToLeaf, self).__init__()
        self.dim = dim

        self.Kp = Kp
        #self.register_buffer('Kp', Kp)

        self.Kv = Kv
        #self.register_buffer('Kv', Kv)

        self.q_des = q_des
        #self.register_buffer('q_des', q_des)

        self.dq_des = dq_des
        #self.register_buffer('dq_des', dq_des)

        self.var = var
        # TODO: 07.10 -> Low variance for joints related with the elbow and big variance for the rest of thew joints
        self.var[2][2] = 10.
        self.var[3][3] = 10.


        ## Multivariate Gaussian distribution ##
        self.p_dx = None

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''
        q = state[0]  # Tensor(7, 1), joint position values
        dq = state[1]  # Tensor (7, 1), joint speed values

        ###########################################
        ddq = self.Kp * (self.q_des - q) + self.Kv * (self.dq_des - dq)

        self.p_dx = tdist.MultivariateNormal(ddq, self.var)  # self.var->torch.size(7, 7)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        # TODO:
        action = action[:, :self.dim]  # torch.Size([1000, 7])
        return self.p_dx.log_prob(action)  # torch.Size([1000])

class JointGoToLeaf_lefthand_and_base(EnergyLeaf):
    def __init__(self, dim=10, Kp = 1., Kv = 1.,
                 q_des=torch.tensor([1.2, 1.0, 0., 2.0, 0.1, 2.0, 1.0, 0.2, -1.0, 0.3]),
                 # ([2.215, 0.088, 2.156, 1.060, 0.238, -1.023, 0.373]),
                 dq_des=torch.tensor([0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]),
                 var=torch.eye(10).float() * 100.):
        super(JointGoToLeaf_lefthand_and_base, self).__init__()

        self.dim = dim

        self.Kp = Kp
        #self.register_buffer('Kp', Kp)

        self.Kv = Kv
        #self.register_buffer('Kv', Kv)

        self.q_des = q_des
        #self.register_buffer('q_des', q_des)

        self.dq_des = dq_des
        #self.register_buffer('dq_des', dq_des)

        self.var = var
        # TODO: 07.10 -> Low variance for joints related with the elbow and big variance for the rest of thew joints
        self.var[0][0] = 10.
        self.var[0][0] = 10.
        # self.var[2][2] = 10.
        # self.var[3][3] = 10.


        ## Multivariate Gaussian distribution ##
        self.p_dx = None

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''
        q = state[0]  # Tensor(7, 1), joint position values
        dq = state[1]  # Tensor (7, 1), joint speed values

        ###########################################
        ddq = self.Kp * (self.q_des - q) + self.Kv * (self.dq_des - dq)

        self.p_dx = tdist.MultivariateNormal(ddq, self.var)  # self.var->torch.size(7, 7)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        # TODO:
        action = action[:, :self.dim]  # torch.Size([1000, 7])
        return self.p_dx.log_prob(action)  # torch.Size([1000])

class PathPlanLeaf_lefthand_and_base(EnergyLeaf):

    def __init__(self, dim=2, Kp = 1., Kv = 1., var=torch.eye(2).float() * 5.):

        super(PathPlanLeaf_lefthand_and_base, self).__init__()
        self.dim = dim

        self.Kp = Kp
        #self.register_buffer('Kp', Kp)

        self.Kv = Kv
        #self.register_buffer('Kv', Kv)

        self.var = var

        ## Multivariate Gaussian distribution ##
        self.p_dx = None

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''
        xy = state[0]  # torch.Size([2]), x and y
        xy_t = torch2numpy(xy).tolist()
        v = state[1]  # torch.Size([2]), dx, dy
        v_t = torch2numpy(v).tolist()

        # TODO: NEED to set a multivariable gaussian distribution of dx. | added on 08.13, 08.17
        ###########################################

        ddx = cascade_control_dx.cascade_control_get_dx(xy_t, v_t)  # TODO: Return n ddx from x points | 09.02
        ddx_t = torch.tensor(ddx)

        self.p_dx = tdist.MultivariateNormal(ddx_t, self.var)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        # TODO:
        action = action[:, :self.dim]  # torch.Size([1000, 2])

        return self.p_dx.log_prob(action)  # torch.Size([1000])

class PathPlanLeaf_lefthand_and_base_np(EnergyLeaf):

    def __init__(self, dim=2, Kp = 1., Kv = 1., var=torch.eye(2).float() * 1.):

        super(PathPlanLeaf_lefthand_and_base_np, self).__init__()
        self.dim = dim

        self.Kp = Kp
        #self.register_buffer('Kp', Kp)

        self.Kv = Kv
        #self.register_buffer('Kv', Kv)

        self.var = var

        ## Multivariate Gaussian distribution ##
        self.p_dx = []

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''
        xy = state[0]  # torch.Size([2]), x and y
        xy_t = np.around(torch2numpy(xy), 3).tolist()
        v = state[1]  # torch.Size([2]), dx, dy
        v_t = np.around(torch2numpy(v), 3).tolist()

        # TODO: NEED to set a multivariable gaussian distribution of dx. | added on 08.13, 08.17
        ###########################################
        ddx = cascade_control_dx.cascade_control_get_n_ddx(xy_t, v_t, num)  # TODO: Return n ddx from x points | 09.02
        ddx_t = torch.tensor(ddx)

        # The N multivariate gaussian distribution

        for i in range(num):
            cur_gaussian = tdist.MultivariateNormal(ddx_t[i], self.var)
            self.p_dx.append(cur_gaussian)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        # TODO:
        action = action[:, :self.dim]  # torch.Size([1000, 2])

        # The summation of n multivariate gaussian distribution
        num = 1
        g = []
        for i in range(num):
            g.append(torch.unsqueeze(self.p_dx[i].log_prob(action), dim=1))

        result = torch.logsumexp(torch.stack(g, dim=2), dim=2).reshape(1000, )
        #result = self.p_dx[0].log_prob(action)
        # g0 = torch.unsqueeze(self.p_dx[0].log_prob(action), dim=1)
        # g1 = torch.unsqueeze(self.p_dx[1].log_prob(action), dim=1)
        # g2 = torch.unsqueeze(self.p_dx[2].log_prob(action), dim=1)
        # result = torch.logsumexp(torch.stack([g0, g1, g2], dim=2), dim=2).reshape(1000, )

        return result

class PathPlanLeaf_pos(EnergyLeaf_x): # TODO: heatmap of position | add 09.13

    def __init__(self, dim=2, Kp = 1., Kv = 1., var=torch.eye(2).float() * 5.):

        super(PathPlanLeaf_pos, self).__init__()
        self.dim = dim

        self.Kp = Kp
        #self.register_buffer('Kp', Kp)

        self.Kv = Kv
        #self.register_buffer('Kv', Kv)

        self.var = var

        ## Multivariate Gaussian distribution ##
        self.p_dx = None

        self.closest_point = None

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''
        xy = state[0]  # torch.Size([2]), x and y
        xy_t = torch2numpy(xy).tolist()
        v = state[1]  # torch.Size([2]), dx, dy
        v_t = torch2numpy(v).tolist()

        # ddx = cascade_control_dx.cascade_control_get_dx(xy_t, v_t)
        # ddx_t = torch.tensor(ddx)

        pos = cascade_control_dx.cascade_control_get_x(xy_t, v_t)
        pos_t = torch.tensor(pos)

        self.closest_point = pos

        closest_points.append(pos)

        self.p_dx = tdist.MultivariateNormal(pos_t, self.var)

    def log_prob(self, action, state):
        """
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        """

        # TODO: add euler discretization | 09.13
        action = action[:, :self.dim]  # torch.Size([1000, 2])

        q_t = state[0]  # torch.Size([2]), x and y
        dq_t = state[1]  # torch.Size([2]), dx, dy

        ddq_t = action
        dq_t = dq_t + ddq_t * 1./240.
        q_t = q_t + dq_t * 1./240.

        # TODO: add heatmap | 09.13
        global path
        global CREATE_DIR
        if not CREATE_DIR:
            path = self.create_dir()
            CREATE_DIR = True

        global count
        if count % 500 == 1:
            grid_map = self.gen_gridmap()
            log_map = torch.exp(self.p_dx.log_prob(grid_map))
            fig = self.gen_heatmap(log_map=log_map, closest_point=self.closest_point)
            self.save_heatmap(fig, path)
        count += 1

        return self.p_dx.log_prob(q_t)  # torch.Size([1000])
        # return self.p_dx.log_prob(action)  # torch.Size([1000])

    def gen_gridmap(self):

        """
            Return a grid map with tensorsize([nc*nr, 2]) for heatmap generation
        """

        dx, dy = -0.01, 0.01

        # generate 2 2d grids for the x & y bounds
        y, x = np.mgrid[1.78:-0.78 + dx:dx, -0.78:1.78 + dy:dy]
        ic(x)
        ic(y)

        nr = len(x)
        nc = len(y)
        grid_map = []
        cur_grid = [0, 0]
        for i in range(nr):
            for j in range(nc):
                cur_grid[0] = x[i][j]
                cur_grid[1] = y[i][j]
                grid_map.append(cur_grid)
                cur_grid = [0, 0]

        grid_map = torch.tensor(grid_map)

        return grid_map

    def gen_heatmap(self, log_map, closest_point):

        """
            log_map: torch.tensorsize([nc* nr, 2])
        """
        fig, ax = plt.subplots(1, 1)  # TODO: Initialize fig
        dx, dy = -0.01, 0.01

        # generate 2 2d grids for the x & y bounds
        y, x = np.mgrid[1.78:-0.78 + dx:dx, -0.78:1.78 + dy:dy]

        nr = len(x)
        nc = len(y)

        log_map = log_map.reshape((nr, nc))

        log_max, log_min = log_map.max(), log_map.min()

        # Mark the goal point
        goal_circle = plt.Circle((1.2, 1.0), 0.01, color='r', fill=True)
        ax.add_patch(goal_circle)

        # Mark the closest point
        closest_circle = plt.Circle((closest_point[0], closest_point[1]), 0.01, color='g', fill=True)
        ax.add_patch(closest_circle)

        c = ax.pcolor(x, y, log_map, cmap='RdBu', vmin=log_min, vmax=log_max)
        ax.set_title('Closest point heatmap')

        fig.colorbar(c, ax=ax)

        fig.tight_layout()
        plt.show()

        return fig

    def create_dir(self):

        t = time.strftime("%Y-%m-%d-%H_%M_%S", time.localtime())

        base_dir = os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        heatmap_dir = os.path.join(base_dir, "Results_figure/heatmap/")
        path = os.path.join(heatmap_dir, t)
        os.mkdir(path)

        return path

    def save_heatmap(self, fig, path):

        t = time.strftime("%Y-%m-%d-%H_%M_%S", time.localtime())

        fig.savefig(path+'/heatmap_{}.png'.format(t), dpi=300)


