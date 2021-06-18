import torch.nn as nn


class EnergyLeaf(nn.Module):
    '''
    An Energy Leaf is an Energy Base Model that provides the unnormalized log prob(action|state)
    '''

    def __init__(self):
        super(EnergyLeaf, self).__init__()

    def set_context(self, state):
        pass

    def log_prob(self, action):
        pass

