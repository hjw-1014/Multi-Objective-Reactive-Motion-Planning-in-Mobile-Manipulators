import torch.nn as nn


class Map(nn.Module):
    '''
    Base Class for Maps
    '''
    def __init__(self):
        super(Map, self).__init__()

    def map_state(self, x):
        return x

    def map_action(self, a):
        return a

