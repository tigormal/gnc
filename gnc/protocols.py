from typing import Iterable, Protocol, Annotated, List, runtime_checkable
from PIL.Image import Image
from coredevice.proto import DeviceProtocol

@runtime_checkable
class NavigationObserverLike(DeviceProtocol, Protocol):

    def setHeadDevice(self, dev):
        ...

    def values(self) -> tuple[list[float], list[float], list[float]]:
        ...

    def position(self) -> list[float]:
        ...

    def velocity(self) -> list[float]:
        ...

    def attitude(self) -> list[float]:
        ...


@runtime_checkable
class TrajectoryGeneratorLike(DeviceProtocol, Protocol):

    def setStartCoords(self, coords: tuple[int, int]):
        ...

    def setEndCoords(self, coords: tuple[int, int]):
        ...

    def setOccupancyGridImage(self, im: Image):
        ...

    def setOccupancyGridMatrix(self, mat: list[list[bool | int]]):
        ...

    def planRoute(self) -> list[Iterable[int | float]]:
        ...


@runtime_checkable
class ManualControlLike(DeviceProtocol, Protocol):

    def setMaxVelocity(self, speed: float):
        ...

    def onButton(self, btnName: str, pressed: bool):
        ...

    def onAxis(self, axisName: str, value: float):
        ...

    @property
    def output(self) -> list[float]:
        ...


@runtime_checkable
class AutomaticControlLike(DeviceProtocol, Protocol):

    def setRoute(self, points: list[Iterable[int | float]]):
        ...

    def setPosition(self, pos: list[float | int]):
        ...

    def setAttitude(self, att: list[float | int]):
        ...

    def setVelocity(self, vel: list[float | int]):
        ...

    def setApproach(self, appr: bool):
        ...

    def setMaxVelocity(self, speed: float):
        ...

    def setTargetDistance(self, distance: float | int):
        ...

    @property
    def output(self) -> list[float]:
        ...


@runtime_checkable
class Mapping2DScannerLike(DeviceProtocol, Protocol):

    def occupancyGridMatrix(self) -> Iterable[Iterable[int]]:
        ...

    def occupancyGridOrigin(self) -> tuple[int|float, int|float]:
        ...

    def occupancyGridSize(self) -> int:
        ...

    def setOccupancyGridSize(self, sizeInM: int):
        ...

    def setOccupancyGridOrigin(self, point: tuple[int, int]):
        ...

    def setPosition(self, pos: list[float | int]):
        ...

    def setAttitude(self, att: list[float | int]):
        ...

    def setVelocity(self, vel: list[float | int]):
        ...

    def setHeadDevice(self, dev):
        ...

    def update(self) -> None:
        ...
