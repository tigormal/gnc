from Pyro5.api import Proxy

gs = Proxy("PYRONAME:gnc.gs")
gs.setMission(
    '''
    TARGET "Alpha/Exit", APPROACH
    '''
)