from coredevice.device import Device
from pydispatch import dispatcher
import logging
from logdecorator import log_on_end, log_on_error, log_on_start
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag # type: ignore
from filterpy.common import Q_discrete_white_noise
import numpy as np
import math
from datetime import datetime
import ahrs
from ahrs import Quaternion
from scipy.spatial.transform import Rotation as R # type: ignore
from Pyro5.api import Proxy
from typing import Any

# TODO: Make this a Device Unit and set log parameters from Guidance
log = logging.getLogger('Navigation.Observer')
# log = logging.getLogger('IMU+UWB Observer')
# file_handler = logging.FileHandler('/tmp/gnc.guidance.log')
# formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)-8s [%(name)s] %(message)s', datefmt="%d-%m-%Y %H:%M:%S",)
# file_handler.setLevel(logging.DEBUG)
# log.addHandler(file_handler)

IMUName = 'IMU'
UWBName = 'UWB'
WBName = 'Wheelbase'

SIGMA_X_UWB = 2.5
SIGMA_X_ACC = 2.5
SIGMA_Y_UWB = 2.5
SIGMA_Y_ACC = 2.5


READ_LENGTH = 1000

def normalize(num, lower=0.0, upper=360.0, b=False):
    """Normalize number to range [lower, upper) or [lower, upper].
    Parameters
    ----------
    num : float
        The number to be normalized.
    lower : float
        Lower limit of range. Default is 0.0.
    upper : float
        Upper limit of range. Default is 360.0.
    b : bool
        Type of normalization. See notes.
    Returns
    -------
    n : float
        A number in the range [lower, upper) or [lower, upper].
    Raises
    ------
    ValueError
      If lower >= upper.
    Notes
    -----
    If the keyword `b == False`, the default, then the normalization
    is done in the following way. Consider the numbers to be arranged
    in a circle, with the lower and upper marks sitting on top of each
    other. Moving past one limit, takes the number into the beginning
    of the other end. For example, if range is [0 - 360), then 361
    becomes 1. Negative numbers move from higher to lower
    numbers. So, -1 normalized to [0 - 360) becomes 359.
    If the keyword `b == True` then the given number is considered to
    "bounce" between the two limits. So, -91 normalized to [-90, 90],
    becomes -89, instead of 89. In this case the range is [lower,
    upper]. This code is based on the function `fmt_delta` of `TPM`.
    Range must be symmetric about 0 or lower == 0.
    Examples
    --------
    >>> normalize(-270,-180,180)
    90
    >>> import math
    >>> math.degrees(normalize(-2*math.pi,-math.pi,math.pi))
    0.0
    >>> normalize(181,-180,180)
    -179
    >>> normalize(-180,0,360)
    180
    >>> normalize(36,0,24)
    12
    >>> normalize(368.5,-180,180)
    8.5
    >>> normalize(-100, -90, 90, b=True)
    -80.0
    >>> normalize(100, -90, 90, b=True)
    80.0
    >>> normalize(181, -90, 90, b=True)
    -1.0
    >>> normalize(270, -90, 90, b=True)
    -90.0
    """
    from math import floor, ceil
    # abs(num + upper) and abs(num - lower) are needed, instead of
    # abs(num), since the lower and upper limits need not be 0. We need
    # to add half size of the range, so that the final result is lower +
    # <value> or upper - <value>, respectively.
    res = num
    if not b:
        if lower >= upper:
            raise ValueError("Invalid lower and upper limits: (%s, %s)" %
                             (lower, upper))

        res = num
        if num > upper or num == lower:
            num = lower + abs(num + upper) % (abs(lower) + abs(upper))
        if num < lower or num == upper:
            num = upper - abs(num - lower) % (abs(lower) + abs(upper))

        res = lower if res == upper else num
    else:
        total_length = abs(lower) + abs(upper)
        if num < -total_length:
            num += ceil(num / (-2 * total_length)) * 2 * total_length
        if num > total_length:
            num -= floor(num / (2 * total_length)) * 2 * total_length
        if num > upper:
            num = total_length - num
        if num < lower:
            num = -total_length - num

        res = num * 1.0  # Make all numbers float, to be consistent

    return res


class AHRS():

	def __init__(self, parent = None):

		self.parent = parent

		self.filterIterations = 6
		self.angles = [0,0,0]
		self.t_old = datetime.now()

		self.aHandle = self.gHandle = self.mHandle = "IMU"

		self.aData = np.empty([3,READ_LENGTH])
		self.gData = np.empty([3,READ_LENGTH])
		self.mData = np.empty([3,READ_LENGTH])
		self.a = []
		self.g = []
		self.m = []
		self.R = np.ndarray(3)
		self._Aw = np.ndarray(3)

		self.q_a = np.array([1.0,0.0,0.0,0.0], dtype=np.float64)

		self.estimator = ahrs.filters.Madgwick(gain_marg = 0.041)
		# self.estimator = ahrs.filters.EKF()

		## Get magnetic declination
		self.wmm = ahrs.utils.WMM() # world magnetic model

		return

	def updateMagneticDeclination(self, latitude: float, longtitude: float, height = 0):
		'''Calculate new magnetic declination for given co-ordinates'''
		self.wmm.magnetic_field(latitude, longtitude, height)
		self.wmm.D

	def update(self, a=[], g=[], m=[]):
		'''Update attitude'''
		ax, ay, az, gx, gy, gz, mx, my, mz = tuple(a+g+m)

		t = datetime.now()
		dt = t - self.t_old
		dt = dt.total_seconds()
		self.t_old = t
		a = np.array([ax, ay, az]) * 9.80665 # m/s^2
		g = np.array([gx, gy, gz]) * np.pi/180.0 # rad/s
		m = np.array([mx, my, mz]) * 0.1 #mT
		afilt = a
		gfilt = g
		mfilt = m
		for _ in range(self.filterIterations):
			self.q_a = self.estimator.updateMARG(q=self.q_a, gyr=gfilt, acc=afilt, mag=mfilt, dt=dt)

		q = Quaternion(self.q_a)
		self.angles = q.to_angles()

		# Get rotation matrix
		r = R.from_quat(self.q_a)

		# Get acceleration in world frame
		self._Aw = np.matmul(r.as_matrix().T, afilt)
		self._dt = dt

		for i in range(len(self.angles)):
			angle = math.degrees(self.angles[i])
			if angle < 0:
				angle += 360
			self.angles[i] = angle
		self.a = afilt
		self.g = gfilt
		self.m = mfilt


class Faith():

	def __init__(self, wb):
		if wb:
			self.wb = wb
			print(f"wheelbase methods: {wb._pyroMethods}")
			self.mL = wb['Motor Left']
			self.mR = wb['Motor Right']
		else:
			raise BaseException("Wheelbase Device Unit is not provided")

		self.h = 0
		self.pos = np.array([45.30, 2.60, 0.0])
		self.vel = np.array([0.0, 0.0, 0.0])
		self.lastTime = datetime.now()

	@log_on_error(logging.ERROR, "Update failed. Reason: {e!r}")
	def update(self):
		t = datetime.now()
		dt = (t - self.lastTime).total_seconds()
		self.lastTime = t

		delta = self.wb.angularVelocity() * dt
		self.h += delta
		normalize(self.h, 0, 360)
		theta = math.radians(self.h)
		vx = self.wb.calcLinearVelocity() * math.cos(theta)
		vy = self.wb.calcLinearVelocity() * math.sin(theta)
		self.pos[0] += vx * dt; self.pos[1] += vy * dt
		self.vel[0] = vx; self.vel[1] = vy
		# speed = np.array([self.mL.speed(), self.mR.speed()])
		# steps = np.mean(np.array([self.mL.steps(), self.mR.steps()]))
		# rps = speed / steps
		# rate = np.pi * self.wheelD * rps
		# deltaHeading = ((rate[0] - rate[1]) / self.wheelDistance) * dt
		# self.heading = self.heading + deltaHeading
		# self.velocity = np.array([np.cos(self.heading), np.sin(self.heading)]) * rate
		# self.velocity = np.append(self.velocity, [0])
		# self.lastPosition = self.lastPosition + self.velocity * dt


class IMU_UWB_WB_Observer():

	def __init__(self, parent: Any, deviceUri: str, sim = False) -> None:
		self.parent = parent
		self.dev = Proxy(deviceUri).assembly()
		self.sim = sim

		# Flags
		self.repHeadFail = False # reported load failure
		self.repDevFail = [False] * 3
		self.devReady = False
		self.filterReady = False
		self.ready = False

		self.pos = [0.0, 0.0, 0.0]
		self.vel = [0.0, 0.0, 0.0]
		self.rot = [0.0, 0.0, 0.0]

		self.load()

	def load(self):
		if not self.devReady: self.prepareDevices()
		if not self.filterReady: self.prepareFilter()

		if self.devReady and self.filterReady: self.ready = True

	def prepareDevices(self):
		# Check if device (proxy) is online and ready
		if self.dev is None:
			log.critical(f'Head Device Unit is not available (None)')
			self.repHeadFail = True
			return
		try:
			self.dev._pyroBind()
			print(self.dev._pyroMethods)
		except Exception as e:
			if not self.repHeadFail:
				log.critical(f'Head Device Unit is not available. Details: {e}')
				self.repHeadFail = True
			return

		self.imu = self.dev[IMUName]
		self.uwb = self.dev[UWBName]
		self.wb = self.dev[WBName]

		for i, dev in enumerate([self.imu, self.uwb, self.wb]):
			if self.sim:
				if i in [0, 1]: continue
			if dev is None:
				log.critical(f"{['IMU', 'UWB', 'Wheelbase'][i]} device not found (is None)")
				self.repDevFail[i] = True
				continue
			try:
				dev._pyroBind()
				# _ = dev.isReady()
			except Exception as e:
				if not self.repDevFail[i]:
					log.critical(f'{dev.name} is not available. Details: {e}')
					self.repDevFail[i] = True
				return

		self.faith = Faith(self.wb)
		self.ahrs = AHRS(self)

		log.info("Devices are prepared")
		self.devReady = True


	def prepareFilter(self):
		self.tracker = KalmanFilter(dim_x=6, dim_z=4)
		self.dt = 0.01

		q = Q_discrete_white_noise(dim=2, dt=self.dt, var=0.05)
		self.tracker.F = np.array([	[1, self.dt, 0.5*self.dt,	0, 0,   	0],
   									[0, 1,   	self.dt,		0, 0,   	0],
   									[0, 0,   	0,  			1, 0,   	0,],
   									[0, 0,   	0,  			1, self.dt, 0.5*self.dt],
   									[0, 0,   	0,  			0, 1,   	self.dt],
   									[0, 0,   	0,  			0, 0,   	1]
									])
		self.tracker.Q = block_diag(q, q)

		self.tracker.H = np.array([[1,0,0,0,0,0],
   								[0,0,1,0,0,0],
   								[0,0,0,1,0,0],
   								[0,0,0,0,0,1]])

		self.tracker.R = np.array([[SIGMA_X_UWB**2, 0, 0, 0],
   								[0, SIGMA_X_ACC**2, 0, 0],
   								[0, 0, SIGMA_Y_UWB**2, 0],
   								[0, 0, 0, SIGMA_Y_ACC**2]]) # type: ignore

		self.tracker.B = np.array([[0, self.vel[0], 0, 0, self.vel[1], 0]]).T # type: ignore
		self.u = np.array([[self.dt]])

		self.tracker.P = np.eye(4) * 500.0
		self.tracker.x = np.array([[0, 0, 0, 0, 0, 0]]).T

		log.info("Filter is prepared")
		self.filterReady = True


	def update(self):
		if self.sim:
			self.faith.update()
			self.pos = list(self.faith.pos)
			self.vel = list(self.faith.vel)
			self.rot = [0.0, 0.0, self.faith.h]

	def values(self) -> tuple[list[float], list[float], list[float]]:
		return self.pos, self.vel, self.rot

	def isReady(self) -> bool:
		return self.ready
