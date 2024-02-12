
from datetime import datetime
# from ipaddress import _N
import logging
import math

from coredevice.gadget import Device
from maps import Map
from gnc.navigation.pos import Position
from gnc.protocols import NavigationObserverLike
from typing import Any, Protocol
from Pyro5.api import expose, oneway, behavior, Proxy
from addict import Dict
import shapely as sh
from pydispatch import dispatcher
from drivers.imu_uwb import IMU_UWB_WB_Observer
from Pyro5.errors import get_pyro_traceback

positionUpdatedEvent = 'positionUpdatedEvent'

## Logging ---
# Create a file handler
file_handler = logging.FileHandler('/tmp/gnc.navigation.log')
# Create a console handler
# console_handler = logging.StreamHandler()
# Configure the handlers
file_handler.setLevel(logging.DEBUG)
# console_handler.setLevel(logging.DEBUG)
# Create a formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)-8s [%(name)s] %(message)s', datefmt="%d-%m-%Y %H:%M:%S",)
file_handler.setFormatter(formatter)
# console_handler.setFormatter(formatter)
log = logging.getLogger('Navigation')
log.addHandler(file_handler)
# log.addHandler(console_handler)
# log.setLevel(logging.DEBUG)
## --- --- ---

posDict = {
            "pos": [0.0, 0.0, 0.0],
            "vel": [0.0, 0.0, 0.0],
            "att": [0.0, 0.0, 0.0]
        }

# class MixedDeviceObserver(Device, NavigationObserverLike): pass

@behavior(instance_mode='single')
@expose
class Navigation():

    def __init__(self, mapPath = '~/Default.map', _kRealInstance=False) -> None:
        log.info('------- Init -------')
        self._currentPosition = posDict
        self._currentPosition["pos"] = [453.0, 26.0, 0.0] # TODO: Temporary start point
        self._ready = False

        self._device = None
        self._mapPath = mapPath
        self._ctrl = None
        self._guid = None

        self._observer: Device = None # type: ignore
        self._map: Proxy


        self._shouldStop = False
        self.failed = False

        self.lastTimeMapSave = datetime.now()

        # Connect events
        # dispatcher.connect(self.updateMapPosition, sender=self, signal=positionUpdatedEvent)

    def currentPosition(self):
        return self._currentPosition

    def updateMapPosition(self):
        ...

    def findInitialPosition(self):
        ...

    def failAck(self):
        self.failed = False

    def main(self):

        log.info('Setting up...')
        while not self.connectSystems(): pass
        if self._observer is not None:
            self._observer.setup()
            self.loadObserver()
            self._observer.start()
            self._observer._ready = True
            

        stillAliveTime = datetime.now()

        log.info('Setup complete')
        log.debug('Entering main loop')
        while True:
            if self._shouldStop: break
            if self._observer is None: continue
            if not self._observer.isReady(): continue
            if self.failed: continue
            try:
                self._observer.update() # type: ignore
                if (datetime.now() - stillAliveTime).total_seconds() > 10:
                    log.debug(f"Still alive")
                    stillAliveTime = datetime.now()
            except Exception as e:
                log.error(f"Couldn't update position. Reason: {e}")
                self.failed = True; continue

            try:
                pos, vel, att = self._observer.values() # type: ignore
                # TODO: Check id data is valid
                self._currentPosition["pos"] = pos
                self._currentPosition["vel"] = vel
                self._currentPosition["att"] = att
            except Exception as e:
                log.error(f"Couldn't obtain position from observer. Reason: {e}")
                self.failed = True; continue

            try:
                if self._ctrl: self._ctrl.setPosition(self._currentPosition)
            except Exception as e:
                log.warning(f"Couldn't send position to Controller. Reason: {e}")
            try:
                if self._guid: self._guid.setPosition(self._currentPosition)
            except Exception as e:
                log.warning(f"Couldn't send position to Guidance. Reason: {e}")
            try:
                if self._map: self._map.setPosition(self._currentPosition)
            except Exception as e:
                log.warning(f"Couldn't send position to Mapping. Reason: {e}")

        # end while
        log.info('Exiting main loop')
        self._map.setPosition(None)

    def setSystemsUri(self, uris:dict):
        self._uris = uris

    def connectSystems(self) -> bool:
        if self.failed: return False

        svars = {
                "Guidance": "_guid",
                # "Navigation": "_nav",
                # "Optical": "_opt",
                "Mapping": "_map",
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
                    self.failed = True
            else:
                if sname != self.__class__.__name__: log.info(f'Skipping {sname} connection')

            if hasattr(self, "_device"): self.connectHeadDevice()

        if self.failed: return False
        return True

    def connectHeadDevice(self):
        if self._ctrl:
            try:
                uri = str(self._ctrl.deviceUri())
                self._device = Proxy(uri)
                self._device._pyroBind()
                log.debug('Head Device Unit connected')
            except Exception as e:
                log.error(f"Couldn't connect to Head Device Unit. Reason: {e}")
                self.failed = True

    # def setObserver(self, obsClass=None):
    #     self.obsClass = IMU_UWB_WB_Observer # Temporary

    def loadObserver(self):
        if self.failed: return
        if self._device is None:
            log.critical("Head Device Unit is unavailable (is None)")
            self.failed = True
            return
        try:
            self._device._pyroClaimOwnership()
        except Exception:
            print("Pyro traceback:")
            print("".join(get_pyro_traceback()))
        if isinstance(self._observer, NavigationObserverLike):
            self._observer.setHeadDevice(self._device.assembly())

    def stopExec(self):
        self._shouldStop = True

    # def setMap(self, map):
    # 	self._map = map

    @property
    def observer(self):
        return self._observer

    @observer.setter
    def observer(self, observer: Device):
        self._observer = observer
        self._observer._genId('Navigation')
        self._observer._getLogger()
        log.info(f"Set navigation observer unit: {observer.name} [{observer.kind}]")
        if not isinstance(observer, NavigationObserverLike):
            log.warning(f"Observer unit '{observer.name}' does not follow NavigationObserverLike protocol. System may not work properly.")
