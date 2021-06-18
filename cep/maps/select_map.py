import torch
import numpy as np

from .base_map import Map
from cep.utils import torch2numpy, numpy2torch
import time


class SelectionMap(Map):
    '''
    Map joints to cartesian space
    '''
    def __init__(self, idx=0):
        super(SelectionMap, self).__init__()
        self.idx = idx

    def map_state(self, x):
        # x[0]->torch.Size([7, 4, 4])
        # x[1]->torch.Size([7, 6])
        xyz = x[0][self.idx, ...] # torch.Size([4, 4])
        v_xyz = x[1][self.idx, ...] # torch.Size([6])
        return [xyz, v_xyz]

    def map_action(self,a):
        return a[:, self.idx, ...]