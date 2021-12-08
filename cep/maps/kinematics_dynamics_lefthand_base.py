import torch
import numpy as np

from .base_map import Map
from cep.utils import torch2numpy, numpy2torch


class FK_ALL_lefthand_base(Map):
    '''
    Map joints to cartesian space
    '''
    def __init__(self, kinematics_model):
        super(FK_ALL_lefthand_base, self).__init__()
        self.kinematics = kinematics_model
        self.J = None

    def map_state(self, x):  # x: torch.Size([1, 20])
        q = x[:, :9]  # torch.Size([1, 10])
        qd = x[:, 9:]  # torch.Size([1, 10])
        q_np = torch2numpy(q[0, :])   # (10, )
        qd_np = torch2numpy(qd[0, :])  # (10, )

        ## Update Kinematics Model ##

        self.kinematics.update_kindyn(q_np)

        z = np.array(self.kinematics.links_fk(rotation=True))  # (10, 4, 4)
        J = np.array(self.kinematics.links_J())  # (10, 6, 10)
        zd = np.einsum('jnm,m->jn', J, qd_np)  # (10, 6) # TODO: ???

        self.J = numpy2torch(J)
        z = numpy2torch(z)  # torch.Size([10, 4, 4])
        zd = numpy2torch(zd)  # torch.Size([10, 6])
        return [z, zd]

    def map_action(self, a):
        return torch.einsum('jnm,bm->bjn', self.J, a)  # torch.Size([1000, 10, 6])
        #  J->torch.Size([10, 6, 10]), a->torch.Size([1000, 10])