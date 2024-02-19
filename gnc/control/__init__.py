from datetime import datetime
import logging
from coredevice.device import Device
from daemonocle import Daemon
from Pyro5.api import callback, expose
from maps import Map, MapObject
from pydispatch import dispatcher
from enum import Enum
from typing import Any, Optional
from coredevice.gadget import Gadget
from Pyro5.api import expose, oneway, behavior, Proxy
import numpy as np
import math
from gnc.protocols import ManualControlLike, AutomaticControlLike
from gnc.guidance import Gamepad

OperationMode = Enum("OperationMode", "AUTO MANUAL")

modeChangedEvent = "modeChangedEvent"


## Logging ---
# Create a file handler
file_handler = logging.FileHandler('/tmp/gnc.controller.log')
# Create a console handler
# console_handler = logging.StreamHandler()
# Configure the handlers
file_handler.setLevel(logging.DEBUG)
# console_handler.setLevel(logging.INFO)
# Create a formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)-8s [%(name)s] %(message)s', datefmt="%d-%m-%Y %H:%M:%S",)
file_handler.setFormatter(formatter)
# console_handler.setFormatter(formatter)

log = logging.getLogger('Controller')
log.addHandler(file_handler)
# log.addHandler(console_handler)
# log.setLevel(logging.DEBUG)
## --- --- ---

class PIDController:
    def __init__(self, Kp, Ki, Kd, offset=0):
        '''
        Initialize the PID controller.

        Args:
            Kp (float): Proportional gain
            Ki (float): Integral gain
            Kd (float): Derivative gain
        '''
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.offset = offset
        self.prev_time = datetime.now()

    def calculate(self, setpoint, feedback_value):
        '''
        Calculate the PID control output.

        Args:
            setpoint (float): The desired value
            feedback_value (float): The actual value

        Returns:
            float: The PID control output
        '''
        error = setpoint - feedback_value
        dt = (datetime.now() - self.prev_time).total_seconds()

        P = self.Kp * error
        self.integral = self.integral + self.Ki * error * dt
        D = self.Kd * (error - self.prev_error) / dt

        output = self.offset + self.integral + P + D

        self.prev_error = error
        self.prev_time = datetime.now()

        return output


@behavior(instance_mode='single')
@expose
class Controller():

    def __init__(self, _kRealInstance=False) -> None:
        log.info('------- Init -------')
        self._shouldStop = False
        self._mode = OperationMode.AUTO
        self._routePoints: list[tuple[int, int]] = []
        self._guid: Proxy
        self._nav: Proxy
        self._map: Proxy
        self._device: Proxy | Gadget | None = None
        self._headConnected = False
        self._appr = False
        self._output = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # velocity vector
        self._failed = False
        self.js = None

        # dispatcher.connect(self.checkCurrentRouteValid, sender=self._map, signal=)
        self.targetVel = 0.1
        self.targetDist = 1.0
        self.startedFollow = False

        self._autoUnit: Optional[AutomaticControlLike] = None
        self._manualUnit: Optional[ManualControlLike] = None

        self.pos = [0.0] * 3
        self.vel = [0.0] * 3
        self.att = [0.0] * 3

    @property
    def failed(self): return self._failed

    @failed.setter
    def failed(self, f):
        log.info(f"Failed is {f}")
        self._failed = f

    def failAck(self):
        self.failed = False

    def setSystemsUri(self, uris:dict):
        self._uris = uris

    def connectSystems(self) -> bool:
        # log.debug("connect systems")
        if self._failed: return False

        svars = {
                "Guidance": "_guid",
                "Navigation": "_nav",
                # "Optical": "_opt",
                "Mapping": "_map",
                # "Controller": "_ctrl"
           }
        # log.debug(str(self._uris))
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

        if self._failed: return False
        return True

    def connectHeadDevice(self):
        try:
            # log.info("Connecting Head device")
            self._device = Gadget.thisGadget()
            if isinstance(self._device, Proxy): self._device._pyroClaimOwnership() # type: ignore
            self._headConnected = True
            log.info("Head device connected")
        except Exception as e:
            log.error(f"Couldn't connect Head Device. Reason: {e}")
            self.failed = True

    @callback
    def headConnected(self) -> bool: return self._headConnected

    @oneway
    def setMode(self, mode: str):
        '''Sets operation mode: `AUTO` or `MANUAL`'''
        log.info(f'Operation mode set to {mode}')
        self._mode = OperationMode[mode]
        dispatcher.send(modeChangedEvent, sender=self)

    @callback
    def mode(self) -> str:
        '''Returns current operation mode'''
        return str(self._mode.name)

    def stopExec(self):
        self._shouldStop = True

    def executeCommand(self, cmd):
        cdd = Proxy("PYRONAME:coredevice.daemon")
        try:
            cdd.executeCommand(cmd)
        except Exception as e:
            log.exception(f"Failed to execute command '{cmd}'")

    @oneway
    def setApproach(self, appr: bool):
        self._appr = appr
        if self._autoUnit is not None: self._autoUnit.setApproach(self._appr)

    @oneway
    def setTargetDistance(self, dist: int | float):
        self.targetDist = dist
        if self._autoUnit is not None: self._autoUnit.setTargetDistance(self.targetDist)

    @oneway
    def setTargetVelocity(self, speed: int | float):
        self.targetVel = speed
        if self._autoUnit is not None: self._autoUnit.setMaxVelocity(self.targetVel)

    def deviceUri(self):
        if isinstance(self._device, Proxy): return self._device._pyroUri
        return ''

    @oneway
    def setPosition(self, d: dict):
        if d:
            self.pos = d["pos"]
            self.vel = d["vel"]
            self.att = d["att"]

            if self._autoUnit is not None:
                self._autoUnit.setPosition(self.pos)
                self._autoUnit.setVelocity(self.vel)
                self._autoUnit.setAttitude(self.att)

    @oneway
    def setRoute(self, points):
        log.info("Got new route")
        self._routePoints = points
        self.startedFollow = False
        log.debug(f"Route len: {len(self._routePoints)}")
        if self._autoUnit is not None: self._autoUnit.setRoute(self._routePoints)

    def connectJoystick(self):
        log.info("Connecting joystick...")
        jsNum = 0
        if hasattr(self._manualUnit, 'jsNumber'): jsNum = self._manualUnit.jsNumber() #type: ignore
        startTime = datetime.now()
        try:
            # log.debug("Connecting joystick...")
            while not Gamepad.available(jsNum):
                if self._shouldStop: return
                if (datetime.now() - startTime).total_seconds() >= 5: break
                if self.js is None:
                    try:
                        jsClass = Gamepad.Gamepad
                        if hasattr(self._manualUnit, 'jsClass'): jsClass = self._manualUnit.jsClass() #type: ignore
                        self.js = jsClass(joystickNumber=jsNum)
                        log.info("Joystick connected")
                        return True
                    except Exception as e:
                        log.error(f"Connecting joystick failed. Reason: {e}")
                        return False
                return False
        except:
            log.exception("Joystick cannot be connected")

    def main(self):
        log.info('Setting up...')

        while not self.connectSystems():
            if self._shouldStop:
                log.debug("should stop is true")
                return

        asm = self._device.assembly()
        if isinstance(asm, Proxy):
            if asm._pyroBind(): log.info("Assembly found") # TODO: catch connection exceptions


        if self._autoUnit is not None:
            log.info("Starting Auto unit")
            try:
                self._autoUnit.setup()
                self._autoUnit.start()
            except:
                log.exception("Auto unit failed")

        if self._manualUnit is not None:
            log.info("Starting Manual unit")
            try:
                self._manualUnit.setup()
                self._manualUnit.start()
            except:
                log.exception("Manual unit failed")

        js = self.connectJoystick()
        if not js: log.warning("Joystick is unavailable. Check connection")

        log.info('Setup complete')
        log.debug('Entering main loop')
        while True:
            if self._shouldStop: break

            if self._manualUnit is not None:
                self._manualUnit.update()
            if self._autoUnit is not None:
                self._autoUnit.update()

            if self._manualUnit and self.js is not None and self.js.isConnected():
                try:
                    eventType, control, value = self.js.getNextEvent()
                    if not (control is None and value is None):
                        if eventType == 'BUTTON':
                            if hasattr(self._manualUnit, "onButton"): self._manualUnit.onButton(str(control), bool(value))
                        elif eventType == 'AXIS' and value is not None:
                            if hasattr(self._manualUnit, "onAxis"):self._manualUnit.onAxis(str(control), value)
                except:
                    log.exception("Allocating joystick event failed.")

            if self._mode == OperationMode.MANUAL and self._manualUnit: self._output = self._manualUnit.output
            elif self._mode == OperationMode.AUTO and self._autoUnit: self._output = self._autoUnit.output

            try:
                if asm and hasattr(asm, "setSpeedVector"): asm.setSpeedVector(self._output) #type: ignore # TODO: switch to protocol check
            except Exception as e:
                log.exception(f"Allocating velocity vector failed.")

        log.info('Exiting main loop')


    def manualMain(self):
        if self._manualUnit is not None:
            self._manualUnit.update()
            self._output = self._manualUnit.output

    def autoMain(self):
        if self._autoUnit is not None:
            self._autoUnit.update()
            self._output = self._autoUnit.output
            # print(self._output)


    @property
    def autoUnit(self):
        return self._autoUnit

    @autoUnit.setter
    def autoUnit(self, u: Device):
        self._autoUnit = u # type: ignore
        # self._autoUnit: AutomaticControlLike
        self._autoUnit._genId('Controller')
        self._autoUnit._getLogger()
        log.info(f"Set automatic control unit: {u.name} [{u.kind}]")
        if not isinstance(u, AutomaticControlLike):
            log.warning(f"Controller unit '{u.name}' does not follow AutomaticControlLike protocol. System may not work properly.")

    @property
    def manualUnit(self):
        return self._manualUnit

    @manualUnit.setter
    def manualUnit(self, u: Device):
        self._manualUnit = u # type: ignore
        # self._manualUnit: ManualControlLike
        self._manualUnit._genId('Controller')
        self._manualUnit._getLogger()
        log.info(f"Set manual control unit: {u.name} [{u.kind}]")
        if not isinstance(u, ManualControlLike):
            log.warning(f"Controller unit '{u.name}' does not follow ManualControlLike protocol. System may not work properly.")
