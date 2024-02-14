from gnc.guidance import Guidance
from gnc.navigation import Navigation
from gnc.control import Controller
from gnc.mapping import Mapping
from maps import Map
from threading import Thread
from Pyro5.api import locate_ns, Proxy, Daemon, oneway, callback, expose
from Pyro5.errors import NamingError
from argparse import ArgumentParser
import daemonocle
import logging
import os, time, socket, sys
from pathlib import Path
from yaml import safe_load, safe_load_all, dump
from typing import Optional, Any
import multiprocessing as mp
from typing import List
from pydoc import importfile
from addict import Dict
from coredevice.device import Device
from coredevice.gadget import (Gadget,
                               defConfigDir,
                               defConfigName,
                               defDriversDir
                               )

# parser = ArgumentParser()
# parser.add_argument("command",
#                     help="[start|stop|reset|restart|status]")
# parser.add_argument("param", nargs='?',
#                     help="command parameters")
# parser.add_argument("-d", "--debug", action="store_true",
#                     help="Do not daemonize")
# parser.add_argument("-v", "--verbose", action="store_true",
#                     help="Show debug output in log file")
# args = parser.parse_args()

log = logging.getLogger('gncd')
logging.basicConfig(
    # format='%(asctime)s.%(msecs)03d %(levelname)-8s [%(name)s] %(message)s',
    format='%(asctime)s.%(msecs)03d %(levelname)s [%(name)s] %(message)s',
    level=logging.DEBUG,
    datefmt="%d-%m-%Y %H:%M:%S",
    handlers=[
            logging.FileHandler('/tmp/gnc.log'),
            logging.StreamHandler()
        ]
)

gncSettingsName = 'gncsettings.yaml'

ports = {'gs': 19901, 'ns': 19902, 'cs': 19903, 'vis': 19905, 'map': 19904, 'gnc': 19900}

class GNCDaemon():

    def __init__(self):

        # self.configPath = Path('~/Devices/Configuration/gnc.yaml').expanduser().resolve() # type: ignore
        self.configPath = Path('~/Devices/Configuration/config.yaml').expanduser().resolve() # type: ignore
        if not self.configPath.is_file():
            raise Exception('Configuration not found')
        with open(self.configPath, 'r') as file:
            self.config = list(safe_load_all(file))
            if len(self.config) >= 2: self.config = self.config[1]

        self._settings = Dict()
        # self.shouldStop = False

        # self.map = Map.load(self.config['Map'])
        units: List[Device] = self.loadUnits(self.config) or []

        self.gs = Guidance(_kRealInstance=True)
        self.cs = Controller(_kRealInstance=True)
        self.ns = Navigation(_kRealInstance=True)
        self.ms = Mapping(_kRealInstance=True)
        self.os = ...

        print(units)
        for u in units:
            match u.name:
                case 'Trajectory Generator':
                    self.gs.planner = u
                case 'Navigation Observer':
                    self.ns.observer = u
                case 'Scanner':
                    self.ms.scanner = u
                case 'Autopilot':
                    self.cs.autoUnit = u
                case 'Manual':
                    self.cs.manualUnit = u
                case 'Optical':
                    # self.os.mlPath = u.handle  # To be implemented
                    ...
                case _:
                    pass

        self._allDevices = [
            self.gs.planner,
            self.ns.observer,
            self.ms.scanner,
            self.cs.autoUnit,
            self.cs.manualUnit,
        ]
        print(self._allDevices)

        self.loadSettings()

        # self.gs.setMap(self.map) # TODO: to be removed
        # self.ms.setMap(self.map)

        # self.gth = mp.Process(target=self.gs.main)
        # self.cth = mp.Process(target=self.cs.main)
        # self.nth = mp.Process(target=self.ns.main)
        # self.mth = mp.Process(target=self.ms.main)
        self.gth = Thread(target=self.gs.main)
        self.cth = Thread(target=self.cs.main)
        self.nth = Thread(target=self.ns.main)
        self.mth = Thread(target=self.ms.main)
        self.gth.name = 'Guidance Thread'
        self.nth.name = 'Navigation Thread'
        self.cth.name = 'Controller Thread'
        self.mth.name = 'Mapping Thread'

        self.threads = [
            self.gth,
            self.cth,
            self.nth,
            self.mth
        ]

        self.nameserv: Proxy
        self.uripath = '/tmp/gnc.'

    def loadUnits(self, unitsDict):
        result = []
        for key, value in unitsDict.items():
            things: List[str] = key.split('[')
            name = things[0].strip()
            kind = ''
            if len(things) > 1: kind = things[1].strip("[ ]")
            driverClass = Device
            try:
                driversDir = Path(defDriversDir)
                driversDir = driversDir.expanduser()
                driversDir = driversDir.resolve()
                path = str(driversDir / kind) + ".py"
                if Path(path).exists():
                    driverModule = importfile(path)
                    driverClass = getattr(driverModule, kind)
                    error = ''
                else:
                    kind = 'Device' # TODO: Return None instead of Device
                    error = f"No driver {kind} found"
                    log.error(error)
            except AttributeError:
                kind = 'Device'
                error = f"Driver module {kind} does not contain driver class"
                log.error(error)
            except ModuleNotFoundError:
                kind = 'Device'
                error = f"No driver {kind} found"
                log.error(error)

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
                subdevices = self.loadUnits(value)
                if subdevices is not None and len(subdevices) > 0:
                    dev = driverClass(subdevices, name=name, kind=kind, extra_dict=None)
                    if dev is not None:
                        result.append(dev)
        return result

    def loadSettings(self):
        config_path = Path(defConfigDir + gncSettingsName)
        config_path = config_path.expanduser()
        config_path = config_path.resolve()
        if config_path.exists():
            config_f = None
            config_d = None
            try:
                config_f = open(config_path, 'r')
            except Exception:
                log.warning('Could not load settings file')
            if config_f is not None:
                config_d = safe_load(config_f)
                config_f.close()
                # print(config_d)  # Check if we imported the desired way
            if config_d is not None:
                self._settings = Dict(config_d)

            # Apply settings to all devices
            if self._allDevices is None:
                return
            for dev in self._allDevices:
                if dev is None: continue
                if self._settings.get(dev._id) is None:
                    self._settings[dev._id] = Dict(dev.defaults())
                dev.defs = self._settings[dev._id]
        else:
            # Create defaults
            log.warning('Could not find settings file. Creating one')
            if self._allDevices is None:
                return
            for dev in self._allDevices:
                self._settings[dev._id] = Dict(dev.defaults())
            self.saveSettings()

    def saveSettings(self):
        config_path = Path(defConfigDir + gncSettingsName)
        config_path = config_path.expanduser()
        config_path = config_path.resolve()
        try:
            settingsDict = self._settings.to_dict()
            with open(config_path, 'w') as yaml_file:
                dump(settingsDict, yaml_file, default_flow_style=False)
            log.debug(f"Settings file saved")
        except Exception as e:
            log.error(f"Could not save settings. Reason: {e}")

    @oneway
    @expose
    def set(self, devId: str, parameter: str, value: Any):
        log.info(f"Settings in {devId}: {parameter} = {value}")
        self._settings[devId][parameter]["val"] = value
        self.saveSettings()
        if self._allDevices:
            for dev in self._allDevices:
                if dev._id == devId:
                    dev.settingsWillChange()
                    dev.defs = self._settings[dev._id]
                    dev.settingsDidChange()

    @callback
    @expose
    def get(self, devId: str, parameter: Optional[str] = None):
        result = self._settings.get(devId) # this will be a dict
        if result and parameter is not None:
            result = result.get(parameter) # get the value if key is provided
            if result:
                result = result.get("val")
        return result


    def locateNameserver(self, quiet=False):
        try:
            self.nameserv = locate_ns()
            return True
        except NamingError as e:
            if not quiet: log.critical(f'Nameserver was not found: {e}')
            return False

    def registerSystems(self, addr, port):
        self.pyrod = Daemon(host=addr, port=port)
        gUri = self.pyrod.register(self.gs)
        cUri = self.pyrod.register(self.cs)
        nUri = self.pyrod.register(self.ns)
        mUri = self.pyrod.register(self.ms)
        with open(self.uripath+'gs.uri', "w") as f:
            f.write(str(gUri))
        with open(self.uripath+'ns.uri', "w") as f:
            f.write(str(nUri))
        with open(self.uripath+'cs.uri', "w") as f:
            f.write(str(cUri))
        with open(self.uripath+'ms.uri', "w") as f:
            f.write(str(mUri))
        self.nameserv.register('gnc.gs', gUri)
        self.nameserv.register('gnc.ns', nUri)
        self.nameserv.register('gnc.cs', cUri)
        self.nameserv.register('gnc.ms', mUri)
        self.pyrodth = Thread(target=self.pyrod.requestLoop)
        self.pyrodth.start()

    def querySystems(self, quiet=False):  # to be called in loop
        gUri = ''
        cUri = ''
        nUri = ''
        mUri = ''
        oUri = ''
        try:
            gUri = self.nameserv.lookup('gnc.gs')
            cUri = self.nameserv.lookup('gnc.cs')
            nUri = self.nameserv.lookup('gnc.ns')
            mUri = self.nameserv.lookup('gnc.ms')
            oUri = self.nameserv.lookup('gnc.os')
        except:
            if not quiet: log.warning(f'Some of systems are not found on the nameserver')
            if not quiet: log.debug(f'List of nameserver registered objects: {self.nameserv.list()}')
        # uris = [gUri, cUri, nUri]
        names = ["Guidance", "Navigation", "Controller", "Mapping", "Optical"]
        uris = {
            names[0]: gUri,
            names[1]: nUri,
            names[2]: cUri,
            names[3]: mUri,
            names[4]: oUri
        }

            # return False
        gs = Proxy(gUri)
        cs = Proxy(cUri)
        ns = Proxy(nUri)
        ms = Proxy(mUri)
        os = None
        sysprox = [gs,ns,cs,ms,os]

        fault = False
        for prox, name in zip(sysprox, names):
            try:
                if prox: prox._pyroBind()
            except:
                if not quiet: log.critical(f'No {name} connection')
                fault = True
        # try:
        # 	gs._pyroBind()
        # except:
        # 	if not quiet: log.critical('No Guidance connection')
        # 	fault = True
        # try:
        # 	cs._pyroBind()
        # except:
        # 	if not quiet: log.critical('No Controller connection')
        # 	fault = True
        # try:
        # 	ns._pyroBind()
        # except:
        # 	if not quiet: log.critical('No Navigation connection')
        # 	fault = True
        if fault: return False

        log.debug("Connecting systems")
        self.ms.setSystemsUri(uris)
        self.gs.setSystemsUri(uris)
        self.ns.setSystemsUri(uris)
        self.cs.setSystemsUri(uris)
        # self.os.setSystemsUri(uris) # TODO: Add optical

        for p in sysprox:
            if p: p._pyroRelease()

        ## Not needed for now
        # with open(self.uripath+'gs.uri', "w") as f:
        # 	f.write(str(gUri))
        # with open(self.uripath+'ns.uri', "w") as f:
        # 	f.write(str(nUri))
        # with open(self.uripath+'cs.uri', "w") as f:
        # 	f.write(str(cUri))

        return True

    def releaseNameserv(self):
        self.nameserv._pyroRelease()

    def startSysThreads(self):
        self.gth.start()
        self.nth.start()
        self.cth.start()
        self.mth.start()

    def stopThreads(self):
        self.gs.stopExec()
        self.ns.stopExec()
        self.cs.stopExec()
        self.ms.stopExec()

    @callback
    @expose
    def getSettingstDict(self):
        return self._settings.to_dict()

    @callback
    @expose
    def getTelemetry(self):
        result = {}
        if self._allDevices:
            for d in self._allDevices:
                result[d._id] = d.telemetry()
                # result[d.name]["id"] = d._id
            return result

    @expose
    def stop(self):
        self.stopThreads()
        self.pyrod.shutdown()
        ns = locate_ns()
        ns.remove(name='gnc.gs')
        ns.remove(name='gnc.ns')
        ns.remove(name='gnc.cs')
        ns.remove(name='gnc.ms')
        ns.remove(name='gnc.os')
        ns.remove(name='gnc.daemon')
        # self.shouldStop = True

def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    res = s.getsockname()[0]
    s.close()
    return res

def daemon_main():
    log.debug('starting executor')
    obj = GNCDaemon()
    port = ports['gnc']
    addr = str(get_ip_address())

    # 1. Locate nameserver
    res = obj.locateNameserver()
    if not res:
        while 1:
            res = obj.locateNameserver(quiet=True)
            if res: break
            time.sleep(1)

    # 2. Register sys objs
    obj.registerSystems(addr, port)

    # 3. Find other sys objs
    res = obj.querySystems()
    if not res:
        while 1:
            res = obj.querySystems(quiet=True)
            if res: break
            time.sleep(1)

    uri=obj.pyrod.register(obj)
    with open("/tmp/gncd.uri", "w") as f:
        f.write(str(uri))
    log.debug(f"Daemon URI: {str(uri)}")
    ns = locate_ns()
    ns.register('gnc.daemon', uri)

    # 4. Start threads
    obj.startSysThreads()

    while 1:
        try:
            obj.pyrod.requestLoop()
        except KeyboardInterrupt:
            obj.stop()
            log.info("Daemon stopped")
            break
        except:
            log.info("Daemon stopped")
            break

class ExecutorDaemonControl(daemonocle.Daemon):

    def findProxy(self):
        uri = ''
        with open('/tmp/gncd.uri', 'r') as f:
            uri = f.read().replace('\n', '')
        if uri == '': uri = "PYRONAME:gnc.daemon"
        self.proxy = Proxy(uri)
        try:
            self.proxy._pyroBind()
            return True
        except Exception:
            print("Daemon is not running")
            return False

    @daemonocle.expose_action
    def start(self, *args, **kwargs):
        log.debug("Calling start")
        # if not self.findProxy():
        #     return
        # self.proxy.start()
        super().start()

    @daemonocle.expose_action
    def stop(self, *args, **kwargs):
        log.debug("Calling stop")
        if not self.findProxy():
            return
        self.proxy.stop()


def main():
    if args.verbose:
        print("Verbose mode")
        log.setLevel(logging.DEBUG)
        logging.getLogger().setLevel(logging.DEBUG)
    else:
        log.setLevel(logging.INFO)
        logging.getLogger().setLevel(logging.INFO)
    if args.debug:
        print("Running in DEBUG mode")
        daemon_main()
    else:
        daemon = ExecutorDaemonControl(
            pid_file='/tmp/gncd.pid',
            stdout_file='/tmp/gnc.log'
        )
        daemon.worker = daemon_main
        daemon.do_action(args.command)

# if __name__ == '__main__':
#     main()

if __name__=='__main__':
    daemon_main()
