from maps import Map, MapObject, MapLayer
import numpy as np
import shapely as sh
from addict import Dict

if __name__ == '__main__':
    map2 = Map.load('Test.map')
    obj = MapObject('Test')
    obj.makePoint([0.0, 0.0])
    map2.moveObjectToLayer(obj, 'Alpha')
    print('SAVE')
    map2.save()