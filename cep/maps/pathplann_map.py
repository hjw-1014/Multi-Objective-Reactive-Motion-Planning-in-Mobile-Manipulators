import torch
import numpy as np

from .base_map import Map
from cep.utils import torch2numpy, numpy2torch
import time


class PathplanMap(Map):  ## TODO: Added on 08.12,
    # TODO: Can try non-holonomic base!!! => change map_state, but it's difficlut | 08.17
    '''
    Map joints to cartesian space
    '''
    def __init__(self, idx=2):
        super(PathplanMap, self).__init__()
        self.idx = idx

    def map_state(self, x):

        xy = x[0][:self.idx]
        v_xy = x[0][10:12]
        return [xy, v_xy]

    def map_action(self, a):

        return a