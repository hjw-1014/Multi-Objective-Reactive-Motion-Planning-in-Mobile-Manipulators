from .base_map import Map


class SimpleBase(Map):
    '''
    Map joints to cartesian space
    '''
    def __init__(self, dim=2):
        super(SimpleBase, self).__init__()
        self.dim = dim


    def map_state(self, x):
        q = x[0]
        qd = x[1]
        return [q, qd]

    def map_action(self, a):
        return a