import shapely as sh
from scipy.spatial.transform import Rotation

class Position():

	def __init__(self, system='xyz') -> None:
		self._q = []
		self._coords = [0.0, 0.0, 0.0]
		self._vel = [0.0, 0.0, 0.0]
		self._quat = [1.0, 0.0, 0.0, 0.0]
		self._sys = system
		self._radius = 0.0 # Estimation error

		self._angles = [0.0, 0.0, 0.0]

	def velocity(self, flat=False):
		n = 2 if flat else 3
		res = self._vel[:n]
		return res

	def setVelocity(self, velocity):
		self._vel[0] = velocity[0]
		self._vel[1] = velocity[1]
		if len(velocity) > 2: self._vel[2] = velocity[2]

	def coords(self, flat=False):
		n = 2 if flat else 3
		res = self._coords[:n]
		return res

	def setCoords(self, coords):
		self._coords[0] = coords[0]
		self._coords[1] = coords[1]
		if len(coords) > 2: self._coords[2] = coords[2]

	def height(self) -> float:
		res = 0
		if self._sys in ['xyz', 'lla']:
			res = self._coords[2]
		elif self._sys == 'nv':
			...
		return res

	def setQuaternion(self, q: list):
		self._quat = q
		rot = Rotation.from_quat(q)
		self._angles = rot.as_euler('xyz', degrees=True)

	def yaw(self):
		return self._angles[2]

	def roll(self):
		return self._angles[0]

	def pitch(self):
		return self._angles[1]

	heading = yaw

	def x(self): return self._coords[0]
	def y(self): return self._coords[1]
	def z(self): return self._coords[2]

