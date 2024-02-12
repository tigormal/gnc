from dataclasses import dataclass
from addict import Dict
from dataclass_wizard import json_field, YAMLWizard
import shapely as sh
from pathlib import Path
from typing import Type, Optional, Union, List

@dataclass
class MapObject(YAMLWizard):
	name: str
	geometry: str | sh.Geometry
	color: int = 0
	highlight: bool = False
	
	def test(self):
		self.geometry = sh.Point(-1.0, 1.0)
		
	@classmethod
	def from_yaml_file(cls, file: str, *, decoder = None, **decoder_kwargs):
		result = super().from_yaml_file(file, decoder=decoder, **decoder_kwargs)
		if hasattr(result, 'geometry'):
			result.geometry = sh.from_wkt(result.geometry)
		return result
		
if __name__ == '__main__':
	point = MapObject('whatever', '')
	point.test()
	point.to_yaml_file('test_1.yaml')
	point2 = MapObject.from_yaml_file('test_1.yaml')
	print(point2)