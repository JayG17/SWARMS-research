# Libraries
import airsimneurips
from dataclasses import dataclass
from typing import List
import time
import numpy as np
from scipy.interpolate import CubicSpline

# Global Variables
client = airsimneurips.MultirotorClient()

# Data Classes
@dataclass
class GateObject:
    gateNumber: int
    x_pos: float
    y_pos: float
    z_pos: float
@dataclass
class GateSphere:
    gateNumber: int
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
@dataclass
class SplineObject:
    gateNumber: str
    waypoints: List[airsimneurips.Vector3r]

# Local Files
from module1 import *
from module2 import *
from module3 import *
from module4 import *
from module5 import *
from module6 import *
from module7 import *