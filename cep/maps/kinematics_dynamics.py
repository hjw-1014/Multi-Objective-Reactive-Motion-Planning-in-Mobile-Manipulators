import torch
import numpy as np

from .base_map import Map
from cep.utils import torch2numpy, numpy2torch


class FK_ALL(Map):
    '''
    Map joints to cartesian space
    '''
    def __init__(self, kinematics_model):
        super(FK_ALL, self).__init__()
        self.kinematics = kinematics_model
        self.J = None

    def map_state(self, x):  # x: torch.Size([1, 14])
        q = x[:, :7]  # torch.Size([1, 7])
        qd = x[:, 7:]  # torch.Size([1, 7])
        q_np = torch2numpy(q[0, :])   # (7, )
        qd_np = torch2numpy(qd[0, :])  # (7, )

        ## Update Kinematics Model ##

        self.kinematics.update_kindyn(q_np)

        z = np.array(self.kinematics.links_fk(rotation=True))  # (7, 4, 4)
        J = np.array(self.kinematics.links_J())  # (7, 6, 7)
        zd = np.einsum('jnm,m->jn', J, qd_np)  # (7, 6) # TODO: ???

        self.J = numpy2torch(J)
        z = numpy2torch(z)  # torch.Size([7, 4, 4])
        zd = numpy2torch(zd)  # torch.Size([7, 6])
        return [z, zd]

    def map_action(self, a):
        return torch.einsum('jnm,bm->bjn', self.J, a)  # torch.Size([1000, 7, 6])
        #  J->torch.Size([7, 6, 7]), a->torch.Size([1000, 7])