from dataclasses import KW_ONLY, dataclass, field, asdict
from addict import Dict
from dataclass_wizard import json_field, YAMLWizard
import shapely as sh
from pathlib import Path
from typing import List, DefaultDict, Any, Optional
from PIL import Image
import os
import  numpy as np
from pydispatch import dispatcher
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from lockfile import Lock

DEFAULT_HEADER_NAME = 'head.yaml'

mapUpdatedEvent = 'mapUpdatedEvent'
mapSavedEvent = 'mapSavedEvent'
mapReloadedEvent = 'mapReloadedEvent'
mapObjectDeletedEvent = 'mapObjectDeletedEvent'
mapLayerDeletedEvent = 'mapLayerDeletedEvent'
mapObjectUpdatedEvent = 'mapObjectUpdatedEvent'
mapLayerUpdatedEvent = 'mapLayerUpdatedEvent'

@dataclass
class MapObject(YAMLWizard):
	name: str
	geometry: str = ''
	color: int = 0
	highlight: Optional[bool] = False # TODO: put highlight and showtext in meta dictionary
	showtext: Optional[bool] = False
	anchors: Optional[dict] = None
	heading: Optional[int] = None
	icon: Optional[str] = None

	layer: Any = json_field('layer', dump=False, default='') # for convenience
	ref: Any = json_field('ref', dump=False, default=None) # reserved for GIS

	def __post_init__(self):
		if self.geometry != '':
			self.geometry = sh.from_wkt(self.geometry)

	def to_dict(self):
		return {k: v for k, v in asdict(self).items() if v is not None}

	def makePoint(self, coords: list):
		self.geometry = sh.Point(coords)
		dispatcher.send(mapObjectUpdatedEvent, sender = self, event={'object': self})

	def makeLine(self, coords: list):
		res = []
		for i, obj in enumerate(coords):
			if isinstance(obj, MapObject):
				if isinstance(obj.geometry, sh.Point):
					if self.anchors is None: self.anchors = {}
					self.anchors[i] = obj.name
					res.append(obj.geometry)
			else:
				res.append(obj)
		self.geometry = sh.LineString(res)
		dispatcher.send(mapObjectUpdatedEvent, sender = self, event={'object': self})

	def makePolygon(self, shell: list):
		res = []
		for i, obj in enumerate(shell):
			if isinstance(obj, MapObject):
				if isinstance(obj.geometry, sh.Point):
					if self.anchors is None: self.anchors = {}
					self.anchors[i] = obj.name
					res.append(obj.geometry)
			else:
				res.append(obj)
		if res[0] != res[-1]:
			res.append(res[0])
		self.geometry = sh.Polygon(res)
		dispatcher.send(mapObjectUpdatedEvent, sender = self, event={'object': self})

	def addHole(self, hole: list):
		if isinstance(self.geometry, sh.Polygon):
			# TODO: Add hole to existing polygon
			...


@dataclass
class MapLayer(YAMLWizard):
	_: KW_ONLY
	objects: List[MapObject] = field(default_factory=list)
	kind: int = 0
	image: str = ''
	hidden: bool = False
	name: str = json_field('name', dump=False, default='') # type: ignore
	size: List[int] = field(default_factory=list) # in meters
	origin: list[int] = field(default_factory=list)
	# meta: dict = field(default_factory=dict)
	source: str = 'm' # 'm' for machine or 'u' for user data source role

	ref: Any = json_field('ref', dump=False, default=None) # reserved for GIS

	def __post_init__(self):
		if self.origin == []:
			self.origin = [0,0]
		if self.size == []:
			self.size = [100,100]
		self._im = None
		self._pixels = None
		self._parent: Any = None

	def to_dict(self):
		return {k: v for k, v in asdict(self).items() if v is not None}

	def object(self, name: str) -> MapObject | None:
		for i in self.objects:
			if i.name == name: return i
		return

	def deleteObjectNamed(self, name: str):
		o = self.object(name)
		if o: self.deleteObject(o)

	def deleteObject(self, obj: MapObject):
		if obj:
			try:
				if obj.ref is not None: obj.ref.delete()
			except:
				pass
			dispatcher.send(mapObjectDeletedEvent, sender = self, event={'object': obj})
			self.objects.remove(obj)

	def imageHandle(self):
		if self._im is None and self.image != '':
			full_path = self.path() / Path(self.image)
			if full_path.exists():
				self._im = Image.open(full_path)
		return self._im

	def path(self) -> Path:
		if self._parent is None:
			return Path()
		pre_path = Path(self._parent.path).expanduser().resolve()
		if not (Path(pre_path)/Path(self.name)).exists():
			os.mkdir(str(Path(pre_path)/Path(self.name)))
		full_path = Path(pre_path)/Path(self.name)
		return full_path

	def save(self):
		if self._im:
			pre_path = Path(self._parent.path) if (self._parent is not None) else Path('')
			if not (Path(pre_path)/Path(self.name)).exists():
				os.mkdir(str(Path(pre_path)/Path(self.name)))
			path = pre_path / Path(self.name) / Path(self.image)
			self._im.save(path, format="TIFF", save_all=True)

	def imageBytes(self) -> np.ndarray:
		'''dtype=np.uint8'''
		return np.array(self._im)

	def setImageBytes(self, name: str, bytes):
		'''dtype=np.uint8'''
		self.image = name
		bytes = np.asarray(bytes, dtype=np.uint8)
		self._im = Image.fromarray(bytes)
		self.save()
		if self._parent is not None:
			self._parent.save()

	def __getitem__(self, item: str):
		return self.object(item)

class MapFileHandler(FileSystemEventHandler):

	def __init__(self, parent):
		self.parent = parent

	def on_modified(self, event):
		if event:
			self.parent.reload()

@dataclass
class Map(YAMLWizard):
	name: str
	path: str = json_field("path", dump=False, default='') # type: ignore
	layers: DefaultDict[str, MapLayer] = field(default_factory=DefaultDict)

	def __post_init__(self):
		self._observer = None
		self._started = False
		self.path = str(Path(self.path).expanduser().resolve())
		for l in self.layers:
			self.layers[l]._parent = self
		# if observe: self.start()

	# TODO: implement event casting for map updates

	def start(self):
		print('obs')
		if self._started: return
		self._observer = Observer()
		self._fshandler = MapFileHandler(self)
		if self.path and self.path != '': 
			try:
				print('start')
				self._observer.schedule(self._fshandler, path=self.path, recursive=True)
				self._observer.start()
			except Exception as e:
				print(f"WARNING: Map file is not observed. Reason: {e}")

	def stop(self):
		...
		if self._observer:
			print('stopping obs')
			self._observer.stop()
			self._observer.join()

	def moveObjectToLayer(self, obj: MapObject, layer: str):
		layer1 = self.layer(obj.layer) if obj.layer != '' else None
		layer2 = self.layer(layer)
		if layer2 is not None:
			obj.layer = layer2.name
			layer2.objects.append(obj)
		if layer1 is not None: layer1.deleteObjectNamed(obj.name)

	def updateObject(self, layer: str, name: str, properties: Dict = Dict()):
		p = properties
		l = self.layer(layer)
		if l is not None:
			o = l.object(name)
			if o is None:
				o = MapObject(name)
				o.layer = layer
				l.objects.append(o)
			o.name = p.name if p.name != Dict() else o.name
			o.color = p.color if p.color != Dict() else o.color
			o.icon = p.icon if p.icon != Dict() else o.icon
			o.geometry = p.geometry if p.geometry != Dict() else o.geometry
			o.heading = p.heading if p.heading != Dict() else o.heading
			o.showtext = p.showtext if p.showtext != Dict() else o.showtext
			o.highlight = p.highlight if p.highlight != Dict() else o.highlight
			if p.layer != Dict() and p.layer != o.layer: self.moveObjectToLayer(o, p.layer)
			dispatcher.send(mapObjectUpdatedEvent, sender = self, event={'object': o})

	def updateLayer(self, layer: str, properties: Dict = Dict()):
		p = properties
		l = self.layer(layer)
		attrs = ['name', 'kind', 'size', 'origin', 'hidden', 'image'] # names of properties that we would want to change
		if l is None: # doesn't exist
			l = MapLayer()
			l._parent = self
			l.name = layer
			self.layers[layer] = l
		for o in l.objects:
			o.layer = l.name
		for key in p:
			if p.name != Dict(): # renaming
				folder = self.layers[layer].path()
				if folder.exists() and str(folder) != '.': 
					folder.rename(Path(self.path) / p.name)
				else:
					folder.mkdir()
				l.name = p.name
				self.layers[l.name] = self.layers.pop(layer)
			else:
				if key in attrs: setattr(l, key, p[key])
				# l.kind = p.kind if p.kind != Dict() else l.kind
				# l.image = p.image if p.image != Dict() else l.image
				# l.origin = p.origin if p.origin != Dict() else l.origin
				# l.hidden = p.hidden if p.hidden != Dict() else l.hidden
		dispatcher.send(mapLayerUpdatedEvent, sender = self, event={'layer': l})

	def deleteLayer(self, layer: str):
		try:
			self.layers.pop(layer)
			dispatcher.send(mapLayerDeletedEvent, sender = self, event={'name': layer})
		except:
			pass

	def deleteObjectNamed(self, layer: str, name: str):
		l = self.layer(layer)
		if l: l.deleteObjectNamed(name)

	def deleteObject(self, obj: MapObject):
		l = self.layer(obj.layer)
		if l: l.deleteObject(obj)

	def save(self):
		if self.path:
			if not Path(self.path).exists():
				os.mkdir(str(Path(self.path)))
			with Lock(self.path):
				self.to_yaml_file(str(Path(self.path) / Path(DEFAULT_HEADER_NAME)))
			dispatcher.send(mapSavedEvent, sender = self)

	def layer(self, name: str) -> MapLayer | None:
		return self.layers.get(name)

	def object(self, layer: str, name: str) -> MapObject | None:
		l = self.layer(layer)
		if l is not None: return l.object(name)

	def reload(self):
		# with Lock(self.path):
		updmap = Map.load(self.path, observe=False)

		# Get new info from layers
		for layerName, layerObject in updmap.layers.items():
			self.updateLayer(layerName, Dict(layerObject.to_dict()))
			for obj in layerObject.objects:
				self.updateObject(layerName, obj.name, Dict(obj.to_dict()))

			# Check if object doesn't exist (was deleted)
			existingLayer = self.layer(layerName)
			if existingLayer is None: continue
			for obj in existingLayer.objects:
				if layerObject.object(obj.name) is None:
					self.deleteObject(obj)

		# Check if a layer doesn't exist (was deleted) in updated map
		for layerName in list(self.layers):
			if updmap.layer(layerName) is None:
				self.deleteLayer(layerName)
		dispatcher.send(mapReloadedEvent, sender = self)



	@classmethod
	def load(cls, path: str, observe=True):
		ppath = Path(path).expanduser().resolve() # type: ignore
		print(f"Load: {ppath}")
		with Lock(ppath):
			header = str(ppath / Path(DEFAULT_HEADER_NAME))
			with open(header, 'r') as f:
				h = f.read()
				# print(h)
				_map = Map.from_yaml(h)
		if isinstance(_map, list):
			_map = _map[0]
		if isinstance(_map, Map):
			_map.path = str(ppath)
		for layerName, layerObject in _map.layers.items():
			layerObject.name = layerName
			for obj in layerObject.objects:
				obj.layer = layerName
		if observe: _map.start()
		return _map

	def __getitem__(self, item: str):
		return self.layers[item]

	def __del__(self):
		self.stop()
