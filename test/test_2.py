import numpy as np
import matplotlib.pyplot as plt
from PIL import Image


layerSize = [42, 42] # meters
layerOrigin = [0,0] # meters (coordinates)
density = 10 # points per pixel

# plt.plot(layerOrigin)

def imageFromBytes(bytes):
    '''dtype=np.uint8'''
    bytes = np.asarray(bytes, dtype=np.uint8)
    return Image.fromarray(bytes)

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
im = imageFromBytes(test_arr)
im.convert('L')

test_arr = np.array(im) / 255
test_arr = test_arr[::-1]


# print(test_arr)
x_coords = []
y_coords = []
xratio, yratio = layerSize[0]/im.width, layerSize[1]/im.height
# for y in range(im.height):
#     for x in range(im.width):
#         # if bytes_from_image[y * im.width + x] != 0:
#         if test_arr[y][x] == 0:
#             x_coords.append((x * layerSize[0]) + layerOrigin[0])
#             y_coords.append((y * layerSize[1]) + layerOrigin[1])
coords = []
for y in range(im.height):
    for x in range(im.width):
        # if bytes_from_image[y * im.width + x] != 0:
        if test_arr[y][x] == 0:
            xx = (x * xratio) + layerOrigin[0] + xratio/(2)
            yy = (y * yratio) + layerOrigin[1] + yratio/(2)
            print(f"({x}, {y}): {(xx, yy)}")
            coords.append((xx, yy))
            x_coords.append(xx)
            y_coords.append(yy)
# x_coords, y_coords = zip(*coords)
# x_coords = list(x_coords)
# y_coords = list(y_coords)[::-1]

         
print(xratio)   
print(yratio)   
print(x_coords)
print(y_coords)

plt.grid(True)
plt.axis("equal")
# plt.plot(test_arr, ".r")
plt.plot(x_coords, y_coords, ".k")

plt.show()
