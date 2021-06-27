from .base_map import Map


class SimplePosVel(Map):
    '''
    Map joints to cartesian space
    '''
    def __init__(self, dim=2):
        super(SimplePosVel, self).__init__()
        self.dim = dim


    def map_state(self, x):
        q = x[0,:self.dim]
        qd = x[0, self.dim:]
        return [q, qd]

    def map_action(self, a):
        return a