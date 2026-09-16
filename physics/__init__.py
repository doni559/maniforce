from math import sqrt, log, pi, sin, cos, radians, degrees
from math import e as euler
from typing import List, Tuple

from dataclasses import dataclass
from pygame import *
from physics.utils import *

from settings import SUBSTEPS, GRAV_CONST, EPS