from simanneal import Annealer

class EzPnP(Annealer):

    def __init__(self, state, grasps):
        self.grasps = grasps
        super(EzPnP, self).__init__(state)

    def move(self):
        # TODO mess with self.state
        pass

    def energy(self):
        # TODO
        return 0