import torch
import torch.distributions as tdist
import numpy as np
import cascade_control_dx
from .energy_leaf import EnergyLeaf
#from _utils import torch2numpy
from cep.utils import eul2rot, rot2eul, rot2quat
from cep.liegroups.torch import SO3, SE3

def torch2numpy(x):
    if x is None:
        print(x)

    if x.device.type=='cuda':
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
            var = torch.eye(self.dim).float() * 0.1
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

    def __init__(self, dim=2, Kp = 1., Kv = 1., var=torch.eye(2).float() * 1.):

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

        ddx = cascade_control_dx.cascade_control_get_dx(xy_t, v_t)
        ddx_t = torch.tensor(ddx)

        self.p_dx = tdist.MultivariateNormal(ddx_t, self.var)  # self.var->torch.size(2, 2)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        # TODO:
        action = action[:, :self.dim]  # torch.Size([1000, 2])
        return self.p_dx.log_prob(action)  # torch.Size([1000])
