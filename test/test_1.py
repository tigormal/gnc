from maps import Map, MapObject, MapLayer
import numpy as np
import shapely as sh

if __name__ == '__main__':
# 	map = Map('Test', 'test/test.map')
# 	map.updateLayer('Test 1', Dict())
# 	print(map.layer('Test 1'))
#
# 	map.updateObject('Test 1', 'Object 1', Dict(geometry = sh.Point(-1, 1)))
# 	map['Test 1'].origin = [1, 1]
# 	print(map.object('Test 1', 'Object 1'))
# 	print(map['Test 1']['Object 1'])
# 	print(map)
# 	map.save()
# 	print('saved')
# 	print()
	map2 = Map.load('Test.map')
	# print(map2)
	# map2.updateLayer('Test 1', Dict(name='Test 1 NEW'))
	# print(map2)
	print()

	test_bytes = [
		[1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
		[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1],
		[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1],
		[1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1],
		[1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1],
		[1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
		[1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1],
		[1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1],
		[1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1],
		[1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1],
		[1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1],
		[1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1],
		[1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1],
		[1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1],
		[1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1],
		[1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1],
		[1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1],
		[1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1],
		[1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1],
		[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
		[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
	]
	test_arr = (1-np.array(test_bytes)) * 255
	# map['Test 1'].setImageBytes('labyrinth.tif', test_arr)
	# print(map['Test 1']._im)
	# map['Test 1'].save()
	im = map2['Test 1'].imageHandle()
	print(im)

	test_point_1 = MapObject('Test Point')
	test_point_1.makePoint([0.0, 1.0])
	print(test_point_1)

	test_point_2 = MapObject('Test Point 2')
	test_point_2.makePoint([1.0, 1.0])
	print(test_point_2)

	test_line_1 = MapObject('Test Line')
	test_line_1.makeLine([test_point_1, test_point_2])
	print(test_line_1)

	test_line_2 = MapObject('Test Line')
	test_line_2.makeLine([sh.Point([1.0, 0.0]), sh.Point([2.0, 3.0])])
	print(test_line_2)

	test_line_3 = MapObject('Test Line')
	test_line_3.makeLine([[4.0, 5.0], [6.0, 7.0]])
	print(test_line_3)
