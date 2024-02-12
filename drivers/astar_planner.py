from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import logging
from PIL.Image import Image
from PIL import ImageFilter, ImageOps
from typing import Iterable
import numpy as np

log = logging.getLogger("Guidance.TrajectoryGenerator")

class AStarPlanner():

    def __init__(self):
        self._matrix = []
        self._image: Image
        self._robotRadius = 0
        self._imageSizeM = 0
        self._start = (0,0)
        self._end = (0,0)
        self.finder = AStarFinder(diagonal_movement=DiagonalMovement.always)

    def setStartCoords(self, coords: tuple[int, int]):
        log.debug(f"Start coords: {coords}")
        self._start = coords

    def setEndCoords(self, coords: tuple[int, int]):
        log.debug(f"End coords: {coords}")
        self._end = coords

    def setOccupancyGridImage(self, im: Image):
        log.debug(f"Occupancy grid image: {im}")
        self._image = im

    def planRoute(self) -> list:
        if self._image is None:
            log.error(f"Trying to plan path with no occupancy grid image")
            return []
        if self._image.width <= 0:
            return []
        # 1. Apply filter
        radius = int(self._robotRadius * self._imageSizeM)
        mf_img = self._image.filter(ImageFilter.MinFilter())

        # # 2. Invert image
        # in_img = ImageOps.invert(mf_img)

        # 3. Covert to array
        matrix = np.array(mf_img)

        # # 4. Set obstacles to zero
        # matrix[matrix == 255] = 0

        # Run path finder
        grid  = Grid(matrix=list(matrix))
        start = grid.node(self._start[0], self._start[1])
        end   = grid.node(self._end[0], self._end[1])

        path, runs = self.finder.find_path(start, end, grid)
        result = []
        for item in path:
            result.append([item.x, item.y])
        log.debug(f"Runs: {runs}, Path len: {len(result)}")

        return result