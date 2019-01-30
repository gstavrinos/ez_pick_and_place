from simanneal import Annealer

class EzPnP(Annealer):

    ez_tools = None

    def __init__(self, state, ez_tools):
        self.ez_tools = ez_tools
        super(EzPnP, self).__init__(state)

    def move(self):
        # TODO mess with self.state
        pass

    def energy(self):
        # TODO
        return 0