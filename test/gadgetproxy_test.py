# from coredevice.gadget import Gadget
from Pyro5.api import Proxy, Daemon, locate_ns, expose
from threading import Thread

from typing import List
# import os
import sys
from pathlib import Path
from coredevice.device import Device
from yaml import load
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader
import logging
from pydoc import importfile
from Pyro5.api import expose, locate_ns, behavior
from Pyro5.client import Proxy

defConfigName = "gadget.yaml"
defCommandsName = "commands.yaml"
defSettingsName = "settings.yaml"
defConfigDir = "~/Devices/Configuration/"
defDriversConfigName = "config.yaml"
defDriversDir = "~/Devices/Drivers/"
defDir = "~/Devices/"

log = logging.getLogger('Gadget')

@behavior(instance_mode='single')
class Gadget:

    config_path = ''
    driver_dir = ''

    # def __new__(cls, _CDRealInstance: bool = False):
    #     if _CDRealInstance:
    #         if not hasattr(cls, 'instance'):
    #             cls.instance = super(Gadget, cls).__new__(cls)
    #         return cls.instance
    #     try:
    #         # remote = Proxy("PYRONAME:coredevice.gadget")
    #         nameserver = locate_ns()
    #         uri = nameserver.lookup("coredevice.gadget")
    #         if uri is not None:
    #             return Proxy(uri)
    #         else:
    #             log.warning("Proxy gagdet object not found. Creating real object instance.")
    #     except Exception:
    #         log.warning("Proxy gagdet object not found. Creating real object instance.")
    #     if not hasattr(cls, 'instance'):
    #         cls.instance = super(Gadget, cls).__new__(cls)
    #     return cls.instance

    def __init__(self, _CDRealInstance: bool = False):
        self._name: str = ''
        self._kind: str = ''
        self._uuid: str = ''
        self._manufacturer: str = ''
        self._model: str = ''
        self._users: List[str] = []
        self._tags: List[str] = []
        self._dashboard: str = ''
        self._telemetry = {}
        self._assembly: Device
        self._assembly = None
        self._load()

    def _load(self):

        config_path = Path(defConfigDir + defConfigName)
        config_path = config_path.expanduser()
        config_path = config_path.resolve()
        if config_path.exists():
            config_f = None
            config_d = None
            try:
                config_f = open(config_path, 'r')
            except Exception:
                log.warning('Could not load gadget file')
            if config_f is not None:
                config_d = load(config_f, Loader)
                config_f.close()
            if config_d is not None:
                self._name = config_d["name"]
                self._kind = config_d["kind"]
                self._tags = config_d["tags"]
                self._model = config_d["model"]
                self._manufacturer = config_d["manufacturer"]

            else:
                log.critical("No gadget configuration is loaded")
        else:
            log.critical("Gadget information not found")
            return

        # if self.config_path == '':
        #     # self.config_path = find_resource_path("config", "config")
        configPath = Path(defConfigDir + defDriversConfigName)
        configPath = configPath.expanduser()
        configPath = configPath.resolve()
        config_f = None
        config_d = None
        self._assembly = Device([])
        if configPath.exists():
            try:
                config_f = open(configPath, 'r')
            except Exception:
                print('Couldnt load config file for gadget')
            if config_f is not None:
                config_d = load(config_f, Loader)
                config_f.close()
                # pprint(config_d)  # Check if we imported the desired way
            if config_d is not None:
                driverClass = Device
                error = False
                if self._model != '':
                    # if self.driver_dir == '':
                    driversDir = Path(defDriversDir)
                    driversDir = driversDir.expanduser()
                    driversDir = driversDir.resolve()
                    dvd = Path(defDir)
                    dvd = dvd.expanduser()
                    dvd = dvd.resolve()
                    sys.path.insert(1, str(dvd))
                    # print(sys.path)
                    driverName = (self._manufacturer+self._model)
                    try:
                        # driverModule = importlib.import_module(driverName)
                        driverModule = importfile(str(driversDir / driverName) + ".py")
                        # print("Loaded head driver module")
                        driverClass = getattr(driverModule, driverName)
                        # print("Loaded driver class")
                        error = None
                    except Exception as e:
                        driverClass = Device
                        error = f"Driver {driverName} not loaded. Reason: {e}"
                        log.error(error)
                        # return
                driverClass = expose(driverClass)
                assemblyDevice = driverClass(self._devicesFromDict(config_d),
                                             name=self._name,
                                             kind=self._kind)
                assemblyDevice._genId('')
                if error:
                    assemblyDevice.report("Initializing device failed")
                    assemblyDevice.report(error)
                self._assembly = assemblyDevice
                print(self._assembly.tree())

            if config_f:
                config_f.close()

    def _devicesFromDict(self, theD) -> List[Device]:
        result = []
        for key, value in theD.items():
            # Split name parts and extract the name and type of the device
            things: List[str] = key.split('[')
            name = things[0].strip()
            kind = things[1].strip("[ ]")
            # print('Device name, kind:', name, kind)

            driverClass = None
            try:
                driversDir = Path(defDriversDir)
                driversDir = driversDir.expanduser()
                driversDir = driversDir.resolve()
                # driverModule = importlib.import_module("Drivers."+kind)
                driverModule = importfile(str(driversDir / kind) + ".py")
                driverClass = getattr(driverModule, kind)
                error = ''
            except AttributeError:
                driverClass = Device
                error = f"Driver module {kind} does not contain driver class"
                log.error(error)
            except ModuleNotFoundError:
                # print("No driver", kind, "found for device", name)
                driverClass = Device
                error = f"No driver {kind} found"
                log.error(error)
            finally:
                driverClass = expose(driverClass)
                print(driverClass)
            #     driverClass = Device

            # Traverse the str, list and dict
            # 1. str - URI or direct handle
            if type(value) == str:
                dev = driverClass(handle=value, name=name, kind=kind, extra_dict=None)
                if dev is not None:
                    result.append(dev)
            # 2. list - first element is a handle, second is an extraDict
            elif type(value) == list:
                uri = value[0]
                extra = None
                if len(value) > 1:
                    extra = value[1]
                dev = driverClass(handle=uri, name=name, kind=kind, extra_dict=extra)
                if dev is not None:
                    result.append(dev)
            # 3. dict - assembly, call this function recursively
            elif type(value) == dict:
                subdevices = self._devicesFromDict(value)
                if subdevices is not None and len(subdevices) > 0:
                    dev = driverClass(subdevices, name=name, kind=kind, extra_dict=None)
                    if dev is not None:
                        result.append(dev)
            # if error != '':
            #     dev.report("Initializing device failed")
            #     dev.report(error)
            #     log.error(f"Problem occured: {error}")
        return result

    @expose
    # @property
    def name(self): 
        return self._name

    @expose
    # @property
    def kind(self): 
        return self._kind

    @expose
    # @property
    def uuid(self): 
        return self._uuid

    @expose
    # @property
    def manufacturer(self): 
        return self._manufacturer

    @expose
    # @property
    def model(self): 
        return self._model

    @expose
    # @property
    def tags(self): 
        return self._tags

    @expose
    # @property
    def users(self): 
        return self._users

    @expose
    # @property
    def dashboard(self): 
        return self._dashboard

    @expose
    # @property
    def telemetry(self): 
        return self._telemetry

    @expose
    # @property
    def assembly(self): 
        return self._assembly



print('start')

gadget = Gadget(_CDRealInstance=True)
# ExposedGadget = expose(Gadget)
# exgadget = ExposedGadget(_CDRealInstance=True)

daemon = Daemon()
dth = Thread(target=daemon.requestLoop)
uri = daemon.register(gadget)
ns = locate_ns()
ns.register("coredevice.gadget", uri)
print('thread start')
dth.start()

print('proxy')
proxy = Proxy("PYRONAME:coredevice.gadget")
proxy._pyroBind()
print(proxy._pyroMethods)
aa = gadget.assembly()
daemon.register(aa)
a = proxy.assembly()
print(proxy)
print(a)
daemon.shutdown()