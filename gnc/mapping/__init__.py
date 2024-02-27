from threading import Lock
from typing import Any
from Pyro5.api import expose, oneway, behavior, Proxy
from addict import Dict
import shapely as sh
from maps import Map, MapObject, MapLayer
from datetime import datetime
import logging, math, time
from pydispatch import dispatcher
from coredevice.gadget import Device, Gadget
import numpy as np
from multiprocessing import shared_memory
from gnc.protocols import Mapping2DScannerLike

## Logging ---
# Create a file handler
file_handler = logging.FileHandler('/tmp/gnc.mapping.log')
# Configure the handlers
file_handler.setLevel(logging.DEBUG)
# Create a formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)-8s [%(name)s] %(message)s', datefmt="%d-%m-%Y %H:%M:%S",)
file_handler.setFormatter(formatter)
log = logging.getLogger('Mapping')
log.addHandler(file_handler)
## --- --- ---

posDict = {
            "pos": [0.0, 0.0, 0.0],
            "vel": [0.0, 0.0, 0.0],
            "att": [0.0, 0.0, 0.0]
        }

positionUpdatedEvent = 'positionUpdatedEvent'

@behavior(instance_mode='single')
@expose
class Mapping():

    def __init__(self, mapPath = '~/Default.map', _kRealInstance=False) -> None:
        log.info('------- Init -------')
        self._currentPosition = posDict

        self._ready = False

        self._map: Map | None = None
        self._mapPath = mapPath
        self._ctrl: Proxy
        self._guid: Proxy
        self._opt: Proxy
        self._nav: Proxy

        self._device: Proxy | Gadget | None = None # Gadget()
        self._scanner: Mapping2DScannerLike | None = None
        self._scannerFailed = False

        self._occGridLayerName = 'Occupancy Grid'
        self._occGridLayerSizePx = 200
        self._occGridLayer = None
        self._occGridShmName = ''
        self._occGridShape = [0,0]
        self._occGridShmCreated = False

        self.shm = None

        self._shouldStop = False
        self._failed = False

        self.lastTimeMapSave = datetime.now()

        # Connect events
        dispatcher.connect(self.updateMapPosition, sender=self, signal=positionUpdatedEvent)


    def updateMapPosition(self):
        if self._map is None: return
        self._map.updateObject(
            'Units', 'This robot', Dict(geometry=sh.Point(self._currentPosition["pos"]), heading=math.degrees(self._currentPosition["att"][2]))
        )

    def stopExec(self):
        self._shouldStop = True

    def setMap(self, map):
        self._map = map

    def setSystemsUri(self, uris:dict):
        self._uris = uris

    def connectSystems(self) -> bool:
        if self._failed: return False

        svars = {
                "Guidance": "_guid",
                "Navigation": "_nav",
                "Optical": "_opt",
                # "Mapping": "_map",
                "Controller": "_ctrl"
           }
        for sname, uri in self._uris.items():

            if sname in list(svars) and uri != '':
                try:
                    setattr(self, svars[sname], Proxy(uri))
                    getattr(self, svars[sname])._pyroBind()
                    log.debug(f'{sname} connected')
                except Exception as e:
                    svars[sname] = None # type: ignore
                    log.error(f"Couldn't connect to {sname}. Reason: {e}")
                    self._failed = True
            else:
                if sname != self.__class__.__name__: log.info(f'Skipping {sname} connection')

        if hasattr(self, "_device"): self.connectHeadDevice()

        if self._failed: return False
        return True

    def connectHeadDevice(self):
        if self._ctrl:
            try:
                while not self._ctrl.headConnected():
                    time.sleep(0.5)
                uri = str(self._ctrl.deviceUri())
                dev = Proxy(uri)
                dev._pyroBind()
                self._device = dev.assembly()
                log.debug('Head Device Unit connected')
                if self._device: log.debug(f"Head Device Unit methods: {self._device._pyroMethods}")
            except Exception as e:
                log.error(f"Couldn't connect to Head Device Unit. Reason: {e}")
                self._failed = True

    def initLayers(self):
        if self._map is None: return
        self._map.updateLayer('Guidance', Dict())
        self._map.updateLayer('Units', Dict())
        self._occGridLayer = self._map.layer(self._occGridLayerName)
        if self._occGridLayer: self._occGridLayer.image = "grid.tif"

    @property
    def failed(self): return self._failed

    @failed.setter
    def failed(self, f):
        log.info(f"Failed is {f}")
        self._failed = f

    def failAck(self):
        self.failed = False
        self._scannerFailed = False

    @oneway
    def setNearestTarget(self, coords: list | tuple | None):
        if self._map is None: return
        if coords is None:
            try:
                self._map.deleteObjectNamed('Guidance', 'Nearest point')
                log.debug(f"Deleted Nearest point object")
            except Exception as e:
                log.debug(f'Deleting Nearest point Object unsuccessful. Reason: {e}')
            return
        # t = MapObject('Target')
        g = sh.Point(coords)
        # self._map.moveObjectToLayer(t, 'Guidance')
        self._map.updateObject('Guidance', 'Nearest point', Dict(geometry=g, color=2))

    @oneway
    def setRouteTarget(self, coords: list | tuple | None):
        if self._map is None: return
        if coords is None:
            try:
                self._map.deleteObjectNamed('Guidance', 'Target')
                log.debug(f"Deleted Target object")
            except Exception as e:
                log.debug(f'Deleting Target object unsuccessful. Reason: {e}')
            return
        g = sh.Point(coords)
        self._map.updateObject('Guidance', 'Target', Dict(geometry=g))

    @oneway
    def setRoute(self, points):
        if self._map is None: return
        if len(points) <=1 or points is None:
            if o := self._map['Guidance']['Route']:
                self._map.deleteObject(o)
                log.debug(f"Deleted Route object")
                return
        try:
            routeMapLine = MapObject('Route', color=1)
            routeMapLine.makeLine(points)
            # Check existing route line object
            if self._map['Guidance']['Route']:
                self._map.updateObject('Guidance', 'Route', Dict(geometry=routeMapLine.geometry))
            self._map.moveObjectToLayer(routeMapLine, 'Guidance')
            log.debug(f"Created Line object: {routeMapLine}")
            # self._map.save()
        except Exception as e:
            log.error(f'Adding route on the map failed. Reason: {e}')
            return

    @oneway
    def setPosition(self, d):
        if self._map is None: return
        if d is not None:
            self._currentPosition = d
            x = d["pos"][0]; y = d["pos"][1]
            yaw = d["att"][2]
            self._map.updateObject(
                'Units', 'This robot', Dict(geometry=sh.Point((x, y)), heading=math.degrees(yaw), icon='Person')
            )
        else:
            self._map.deleteObjectNamed('Units', 'This robot')
        # self._map.save()

    def getMapPointCoords(self, layer, name) -> list:
        if self._map is None: return []
        l = self._map[layer]
        if l:
            o = l[name]
            if o and o.geometry:
                return [o.geometry.x, o.geometry.y] #type: ignore
        return []

    def createShm(self):
        log.debug('Create shared mem')
        if not self._map:
            log.critical('Attempting to build route with invalid map')
            log.debug(f'Map object: {self._map}')
            return
        layer = self._map[self._occGridLayerName]
        if not layer:
            log.critical('Attempting to build route with invalid layer')
            log.debug(f'Layer name: {self._occGridLayerName}, object: {layer}')
            return
        im = layer.imageHandle()
        if not im:
            log.warning('Attempting to build route with layer without image')
            log.debug(f'Layer name: {self._occGridLayerName}, object: {layer}')
        else:
            grid = np.zeros((self._occGridLayerSizePx, self._occGridLayerSizePx), dtype=np.uint8)
            layer.setImageBytes(grid)

        image_array = np.array(im)
        try:
            # Create a new shared memory block
            self.shm = shared_memory.SharedMemory(create=True, size=image_array.nbytes)
            # Create a numpy array backed by shared memory
            self.shmim = np.ndarray(image_array.shape, dtype=image_array.dtype, buffer=self.shm.buf)
            # Copy the local array to shared memory
            np.copyto(self.shmim, image_array)
            # Return the name of the shared memory block
            self._occGridShmName = self.shm.name
            self._occGridShape = list(image_array.shape)
            log.debug(f"Created shared mem with name: '{self._occGridShmName}'")
        except Exception as e:
            log.error(f'Failed to create shareable image array. Reason: {e}')
            return ''
        try:
            self._guid.resetShm()
            self._guid.attachShm(self._occGridShmName, self._occGridShape)
        except Exception as e:
            log.error(f'Failed to notify Guidance of new shared mem. Reason: {e}')
            return self._occGridShmName

    def resetShm(self):
        self._occGridShmName = ''
        self._occGridShape = [0,0]
        self.shmim = np.array([])
        if self.shm:
            self.shm.close()
            self.shm.unlink()
        self.shm = None

    def getOccupancyGridShmName(self) -> str:
        return self._occGridShmName

    def getOccupancyGridShape(self) -> list[int]:
        return self._occGridShape

    def getOccupancyGridMatrix(self) -> list[int]:
        return list(self.shmim)


    def getLayerInfo(self):
        if self._map is None: return {}
        layer = self._map[self._occGridLayerName]
        if not layer:
            log.critical('Attempting to build route with invalid layer')
            log.debug(f'Layer name: {self._occGridLayerName}, object: {layer}')
            return []
        return {
            "size": layer.size,
            "origin": layer.origin
        }

    def getMapLine(self, layer, name) -> list:
        if self._map is None: return []
        l = self._map[layer]
        if l:
            o = l[name]
            if o and o.geometry:
                return list(o.geometry) #type: ignore
        return []

    def saveMap(self):
        if self._map is None: return
        if (datetime.now() - self.lastTimeMapSave).total_seconds() >= 5:
            with Lock():
                try:
                    log.debug(f"Writing to map file. \n\tPosition: {self._currentPosition['pos']}")
                    if self._occGridLayer: self._occGridLayer.setImageBytes(self.shmim)
                    self._map.save()
                except Exception as e:
                    log.exception("Saving map failed")
            self.lastTimeMapSave = datetime.now()

    def main(self):
        log.info('Setting up...')
        while not self.connectSystems(): pass

        while not self._map:
            try:
                if self._shouldStop: return
                if not self._failed: self._map = Map.load(self._mapPath)
                time.sleep(0.5)
            except Exception as e:
                log.error(f"Failed to load map file. Reason: {e}")
                self._failed = True

        if self._scanner is not None:
            try:
                if self._occGridLayer:
                    self._scanner.setOccupancyGridSize(self._occGridLayer.size[0])
                    self._scanner.setOccupancyGridOrigin((self._occGridLayer.origin[0], self._occGridLayer.origin[1]))
                self._scanner.setup()
                self._scanner.setHeadDevice(self._device)
                self._scanner.start()

                # orig = self._scanner.occupancyGridOrigin()
                # size = self._scanner.occupancyGridSize()
                # if orig and size: self._map.updateLayer('Occupancy Grid', properties=Dict(origin = list(orig), size = list(size)))
            except:
                log.exception("Scanner failed")
                self._scannerFailed = True

        self.createShm()

        log.info('Setup complete')
        log.debug('Entering main loop')
        while True:
            if self._shouldStop: break
            if self.shm is None: self.createShm()
            self.saveMap()
            if self._scanner and not self._scannerFailed:
                try:
                    self._scanner.update()
                    im = self._scanner.occupancyGridMatrix()
                    if im is not None: np.copyto(self.shmim, np.array(im))
                except:
                    log.exception("Scanning failed")
                    self._scannerFailed = True

        log.info('Exiting main loop')
        self.resetShm()


    @property
    def scanner(self):
        return self._scanner

    @scanner.setter
    def scanner(self, scanner: Device | None):
        if scanner is None:
            log.warning(f"No Scanner unit provided")
            return
        self._scanner = scanner  # type: ignore
        self._scanner._genId('Mapping')
        self._scanner._getLogger()
        log.info(f"Set scanner unit: {scanner.name} [{scanner.kind}]")
        if not isinstance(scanner, Mapping2DScannerLike):
            log.warning(f"Scanner unit '{scanner.name}' does not follow Mapping2DScannerLike protocol. System may not work properly.")
