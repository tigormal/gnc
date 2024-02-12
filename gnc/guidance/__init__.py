import logging
from coredevice.device import Device
from daemonocle import Daemon
from Pyro5.api import expose, Proxy, locate_ns, behavior, oneway
from maps import Map, MapObject
from pydispatch import dispatcher
from enum import Enum
from typing import Any
import os, time, socket, sys
import shapely as sh
from addict import Dict
import numpy as np
from PIL import Image
from multiprocessing import shared_memory

# from gnc.guidance.astar import BidirectionalAStarPlanner
from gnc.guidance.lex import parse
# from drivers.astar_planner import AStarPlanner
from gnc.protocols import TrajectoryGeneratorLike


MissionState = Enum("MissionState", "unset set en_route failed")

routeUpdatedEvent = "routeUpdatedEvent"
missionChangedEvent = "missionChangedEvent"
mapUpdatedEvent = 'mapUpdatedEvent'
stepFinishedEvent = "stepFinishedEvent"
shmResetEvent = 'shmResetEvent'

## Logging ---
# Create a file handler
file_handler = logging.FileHandler('/tmp/gnc.guidance.log')
# Create a console handler
console_handler = logging.StreamHandler()
# Configure the handlers
file_handler.setLevel(logging.DEBUG)
# console_handler.setLevel(logging.INFO)
# Create a formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)-8s [%(name)s] %(message)s', datefmt="%d-%m-%Y %H:%M:%S",)
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)
log = logging.getLogger('Guidance')
log.addHandler(file_handler)
# log.addHandler(console_handler)
log.setLevel(logging.DEBUG)
# log.propagate = False
## --- --- ---

class MissionStep():
    TARGET: str = ''
    COMMAND: str = ''
    FOLLOW: str = ''
    VISUAL: str = ''
    DISTANCE: int = 0
    SPEED: int = 0
    AREA: str = ''
    APPROACH: bool = False
    SHIFT: list = [0,0,0]
    ABORT = False

@behavior(instance_mode='single')
@expose
class Guidance():

    def __init__(self, mapPath = '~/Default.map', _kRealInstance=False) -> None:
        log.info('------- Init -------')
        self._mission: str = ''
        self._prog = {}
        self._steps: list[MissionStep | None] = []
        self._state = MissionState.unset
        self._shouldStop = False

        self._routePoints: list[tuple[int, int]] = []
        self._planner = None #AStarPlanner()

        self._ctrl: Proxy
        self._nav: Proxy
        self._opt: Proxy
        self._map: Proxy

        self._mapmap: Map

        self._ctrlUri: str = ''
        self._navUri: str = ''
        self._optUri: str = ''

        # self._missionLexer = None
        # self._missionParser = None
        self._jsController = ...

        self._occGridLayerName = 'Occupancy Grid'
        self._guidLayerName = 'Guidance'

        self.shm = None
        self.shm_name = ''
        self.matrix = [] # occ grid
        self._pos = [0, 0]
        self._targetCoords: list[int] = []
        self._targetVel = 0
        self._approach = False
        self._area = sh.Polygon()
        self.failed = False

    @oneway
    def setPosition(self, d: dict):
        if d: self._pos = [d["pos"][0], d["pos"][1]]

    @oneway
    def setMission(self, missionText: str):
        '''Sets mission text'''
        log.info('Mission set')
        self._mission = missionText
        try:
            self._prog = parse(self._mission)
            self.convertProgToStep()
            self._state = MissionState.set
            dispatcher.send(missionChangedEvent, sender=self)
        except Exception as e:
            log.error(f"Invalid mission: {e}")
            self._state = MissionState.failed

    def convertProgToStep(self):
        res = MissionStep()
        if isinstance(self._prog, dict):
            for key, value in self._prog.items():
                setattr(res, key, value)
            self._steps = [res]

    def mission(self) -> str:
        '''Returns current mission text'''
        return self._mission

    def missionState(self) -> str:
        '''Returns current mission state: unset, set, en-route or failed'''
        return str(self._state.name)

    def takeNextStep(self) -> MissionStep | None:
        '''Switches to the next step in the mission. Returns new step'''
        log.info('Next step')
        if len(self._steps) > 0:
            dispatcher.send(stepFinishedEvent, sender=self)
            return self._steps.pop()

    def currentStep(self) -> MissionStep | None:
        '''Returns current step'''
        return self._steps[0] if len(self._steps)>0 else None

    def malformedStep(self):
        self._steps[0] = MissionStep()


    def failAck(self):
        self.failed = False

    def setSystemsUri(self, uris:dict):
        self._uris = uris

    def notifyMapUpdated(self): dispatcher.send(signal=mapUpdatedEvent, sender=self)

    def connectSystems(self) -> bool:
        if self.failed: return False

        svars = {
                # "Guidance": "_guid",
                "Navigation": "_nav",
                "Optical": "_opt",
                "Mapping": "_map",
                "Controller": "_ctrl"
           }
        for sname, uri in self._uris.items():

            if sname in list(svars) and uri != '':
                try:
                    setattr(self, svars[sname], Proxy(uri))
                    getattr(self, svars[sname])._pyroBind() #type: ignore
                    log.debug(f'{sname} connected')
                except Exception as e:
                    svars[sname] = None # type: ignore
                    log.error(f"Couldn't connect to {sname}. Reason: {e}")
                    self.failed = True
            else:
                if sname != self.__class__.__name__: log.info(f'Skipping {sname} connection')

            if hasattr(self, "_device"): self.connectHeadDevice()

        if self.failed: return False
        return True

    def connectHeadDevice(self):
        try:
            if isinstance(self._device, Proxy): self._device._pyroClaimOwnership() # type: ignore
        except Exception as e:
            log.error(f"Couldn't reclaim Gadget proxy. Reason: {e}")
            self.failed = True

    def checkCurrentRouteValid(self, event=None):
        '''Check whether the current routes geometry was untouched and the route should be recalculated'''
        step = self.currentStep()
        if step:
            if step.TARGET == '': self._routePoints.clear()
            else:
                targetSymbols = self.mapSymbolsFromName(step.TARGET)
                if len(self._targetCoords) and len(targetSymbols) == 2: self._routePoints.clear()


    def stopExec(self):
        self._shouldStop = True

    def attachShm(self, name='', shape=[]):
        log.debug(f"Attach shared mem: '{name}' of size {shape}")
        self.shm_name = name #self._map.getOccupancyGridShmName()
        self.matrix_shape = shape #self._map.getOccupancyGridShape()
        if self.shm_name == '' or self.shm_name is None:
            log.debug(f"Got empty shared mem name")
            return
        if self.matrix_shape is None or len(self.matrix_shape)<=1:
            log.debug(f"Got empty shared mem array shape")
            return
        self.shm = shared_memory.SharedMemory(name=self.shm_name)
        self.matrix = np.ndarray(self.matrix_shape, dtype=bool, buffer=self.shm.buf)

    def resetShm(self):
        log.debug(f"Reset shared mem")
        self.shm_name = ''
        if self.shm: self.shm.close()
        self.shm = None
        dispatcher.send(shmResetEvent, self)

    #! Route planning
    def planRoute(self):
        log.debug('Plan route')
        ## Checks for everything ready
        self._state = MissionState.failed
        # if not self._mapmap:
        # 	log.critical('Attempting to build route with invalid map')
        # 	log.debug(f'Map object: {self._mapmap}')
        # 	return
        # layer = self._mapmap[self._occGridLayerName]
        # if not layer:
        # 	log.critical('Attempting to build route with invalid layer')
        # 	log.debug(f'Layer name: {self._occGridLayerName}, object: {layer}')
        # 	return
        # im = layer.imageHandle()
        # if not im:
        # 	log.critical('Attempting to build route with layer without image')
        # 	log.debug(f'Layer name: {self._occGridLayerName}, object: {layer}')
        # 	return
        # if self._target is None or not isinstance(self._target.geometry, sh.Point):

        # if self.shm is None: self.attachShm()

        if self.matrix is None or len(self.matrix) == 0:
            log.critical('Occupancy grid is empty. Unable to plan route')
            return

        layer = self._map.getLayerInfo()
        if layer is None:
            log.critical('No layer information provided. Unable to plan route')
            return

        if not len(self._targetCoords) >= 2 :
            log.critical('Attempting to build route with invalid target')
            log.debug(f'Target: {self._targetCoords}')
            return
        self._state = MissionState.set

        im = Image.fromarray(self.matrix)

        ratio = 100 * layer["size"][0] / im.width
        start = np.array([
            (self._pos[0] + layer["origin"][0]) * ratio,
            (self._pos[1] + layer["origin"][1]) * ratio
        ])
        end = np.array([
            (self._targetCoords[0] + layer["origin"][0]) * ratio,
            (self._targetCoords[1] + layer["origin"][1]) * ratio
        ])
        sround = np.floor(start)
        eround = np.floor(end)

        delta = np.abs(sround-start)

        if self._planner is None: return
        self._planner.setStartCoords(tuple(sround.astype(int))) # type: ignore
        self._planner.setEndCoords(tuple(eround.astype(int))) # type: ignore
        if im: self._planner.setOccupancyGridImage(im) # type: ignore

        try:
            path = self._planner.planRoute() # type: ignore
        except:
            path = []
        if len(path) <= 1:
            log.warning("Failed to plan path. Destination is unreachable")
            self.malformedStep()
            return
        else:
            log.info("Found path")
        path = np.array(path)
        # for i in range(len(path)):
        # 	path[i][0] = path[i][0] * im.width / layer.size[0]
        # 	path[i][1] = path[i][1] * im.width / layer.size[0]
        # self._routePoints = path
        path = path * im.width / (100 * layer["size"][0])
        # pathcorr = path + (path*delta)
        self.setRoute(path.astype(float).tolist())



        self._state = MissionState.en_route

    def setRoute(self, points):
        log.debug(f'Set route: {str(points)}')
        self._routePoints = list(points)
        dispatcher.send(routeUpdatedEvent, sender=self)
        ## Add route onto map
        if len(points) <=1:
            self._map.setRoute([])
            # self._map.deleteObjectNamed(self._guidLayerName, 'Route')
            # self._map.save()
            # log.debug(f"Deleted Line object (Route)")
            return


    def mapSymbolsFromName(self, name):
        s = name.split('/')
        res = []
        for word in s:
            word = word.strip("\"")
            word = word.strip()
            res.append(word)
        return res

    def setMap(self, map):
        self._mapmap = map

    def notifyController(self):
        self._ctrl.setRoute(self._routePoints)
        self._ctrl.setTargetVelocity(self._targetVel)
        self._map.setRoute(self._routePoints)


    #! Main
    def main(self):
        log.info('Setting up...')
        shouldPlan = False

        while not self.connectSystems(): pass

        if self._planner is not None:
            self._planner.setup()
            self._planner.start()

        # if self.shm is None: self.attachShm()

        # self._map.updateLayer('Guidance', Dict())
        # self._map.save()
        d = self._nav.currentPosition()
        if d: self._pos = d.get("pos") or [0, 0]
        # Connect events
        dispatcher.connect(self.checkCurrentRouteValid, sender=self, signal=missionChangedEvent)
        dispatcher.connect(self.checkCurrentRouteValid, sender=self, signal=mapUpdatedEvent)
        # dispatcher.connect(self.checkCurrentRouteValid, sender=self._map, signal='mapObjectUpdatedEvent')
        # dispatcher.connect(self.checkCurrentRouteValid, sender=self._map, signal='mapReloadedEvent')
        dispatcher.connect(self.notifyController, sender=self, signal=routeUpdatedEvent)
        # dispatcher.connect(self.attachShm, sender=self, signal=shmResetEvent)

        log.info('Setup complete')
        log.debug('Entering main loop')
        while True:
            if self._shouldStop: break

            if len(self._steps) == 0:
                if len(self._routePoints) > 1:
                    self.setRoute([])
                continue

            step = self.currentStep()
            if step is None: continue

            #! 1. Interpret step from mission
            if step.ABORT:
                log.info('Abort mission')
                self.setRoute([])
                shouldPlan = False
                self._steps.clear()

            if step.SPEED >= 0:
                # log.info(f'Speed set to: {step.SPEED}')
                self._targetVel = step.SPEED

            if step.TARGET != '':
                # log.debug('Target processing')
                def applyShift(coords: list, shift: list):
                    log.debug('Apply shift')
                    # coords = list(obj.geometry)
                    for i in range(2):
                        coords[i] = coords[i] + shift[i]
                    # obj.makePoint(coords)
                    return coords

                if len(self._routePoints) <= 1: # Not built route yet
                    point = sh.from_wkt(step.TARGET, on_invalid='ignore')
                    if point is None:
                        log.debug('Getting point from map')
                        # Try getting map object
                        targetSymbols = self.mapSymbolsFromName(step.TARGET)
                        log.debug(f'Symbols: {targetSymbols}')
                        if len(targetSymbols) == 2:
                            # self._target = self._mapmap[targetSymbols[0].strip()][targetSymbols[1].strip()]
                            self._targetCoords = self._map.getMapPointCoords(targetSymbols[0].strip(), targetSymbols[1].strip()) or []
                            if len(self._targetCoords) >= 2:
                                shouldPlan = True
                                if step.SHIFT[0] != 0 or step.SHIFT[1] != 0 or step.SHIFT[2] != 0:
                                    # self._target = MapObject('Target', geometry=self._target.geometry)
                                    self._targetCoords = applyShift(self._targetCoords, step.SHIFT)
                                    self._map.setRouteTarget(self._targetCoords)
                                    # self._map.moveObjectToLayer(self._target, self._guidLayerName)
                                    # self._map.save()
                        else:
                            log.error(f'Given malformed map object name: {step.TARGET}')
                            self.malformedStep() # Hang until further instructions

                    elif isinstance(point, sh.Point):
                        log.debug('Creating temporary map object')
                        # Valid geometry, create temporary MapObject
                        # self._target = MapObject('Target')
                        # self._target.geometry = point
                        self._targetCoords = applyShift(self._targetCoords, step.SHIFT)
                        self._map.setRouteTarget(self._targetCoords)
                        # self._map.moveObjectToLayer(self._target, self._guidLayerName)
                        # self._map.save()
                        shouldPlan = True
                    else:
                        log.error(f'Given wrong geometry for TARGET (not a Point)')
                        self.malformedStep() # Hang until further instructions


            if step.COMMAND != '':
                log.debug('Command processing')
                if self._ctrl: self._ctrl.executeCommand(step.COMMAND)

            if step.FOLLOW != '':
                # log.debug('Follow processing')
                routeSymbols = step.TARGET.split('/')
                if not (len(routeSymbols) < 2):
                    # line = self._mapmap[routeSymbols[0].strip()][routeSymbols[1].strip()]
                    line = self._map.getMapLine(routeSymbols[0].strip(), routeSymbols[1].strip())
                    if line: self._routePoints = line
                    # if line is not None and isinstance(line.geometry, sh.LineString):
                    # 	self._routePoints = list(line.geometry)
                else:
                    ... # TODO: syntax error

            if step.APPROACH:
                # log.debug('Approach processing')
                self._approach = True
                # if ctrl: ctrl.setApproach(True)

            #! 2. Target
            ## Check if route is plotted
            if len(self._routePoints) <= 1 and shouldPlan:
                self.planRoute()

            #! 3. Check events
            ...

            #! 4. Check destination
            if self._targetCoords:
                # targetPoint = self._target.geometry
                targetPoint = sh.Point(self._targetCoords)
                positionPoint = sh.Point(self._pos)
                dist = sh.distance(targetPoint, positionPoint)
                if dist <= step.DISTANCE:
                    self.takeNextStep() # Reached target, moving on

        # while end
        log.info('Exiting main loop')
        self.resetShm()




    #! Proxy and daemon stuff

    # def __new__(cls, _kRealInstance: bool = False):
    # 	defProxyname = "PYRONAME:guidance.gnc.obj"

    # 	if _kRealInstance:
    # 		if not hasattr(cls, 'instance'):
    # 			cls.instance = super(Guidance, cls).__new__(cls)
    # 		return cls.instance
        # with open('/tmp/guidance.gnc.uri', 'r') as f:
        # 	uri = f.read().replace('\n', '')
        # if uri == '':
        # 	uri = defProxyname
        # else:
        # 	return Proxy(uri)
        # try:
        # 	nameserver = locate_ns()
        # 	uri = nameserver.lookup("guidance.gnc.obj")
        # 	if uri is not None:
        # 		return Proxy(uri)
        # 	else:
        # 		log.warning("Proxy object not found. Creating real object instance.")
        # except Exception:
        # 	log.warning("Proxy object not found. Creating real object instance.")
        # if not hasattr(cls, 'instance'):
        # 	cls.instance = super(Guidance, cls).__new__(cls)
        # return cls.instance

    @classmethod
    def _kRealInstance(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(Guidance, cls).__new__(cls)
        return cls.instance

    @property
    def planner(self):
        return self._planner

    @planner.setter
    def planner(self, planner: Device):
        self._planner = planner
        self._planner._genId('Guidance')
        self._planner._getLogger()
        log.info(f"Set trajectory generator unit: {planner.name} [{planner.kind}]")
        if not isinstance(planner, TrajectoryGeneratorLike):
            log.warning(f"Trajectory generator unit '{planner.name}' does not follow TrajectoryGeneratorLike protocol. System may not work properly.")
