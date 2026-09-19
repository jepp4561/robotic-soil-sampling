from dataclasses import dataclass
from math import atan2
from math import cos
from math import degrees
from math import log
from math import pi
from math import sin
from math import sqrt
from statistics import mode
from typing import Sequence


@dataclass
class Statistics:
    mean: float
    stddev: float


def calculate_statistics(values: Sequence[float]) -> Statistics:
    if not values:
        raise ValueError("Cannot calculate statistics from empty data.")

    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)

    return Statistics(mean=mean, stddev=sqrt(variance))


def calculate_circular_statistics(values: Sequence[float]) -> Statistics:
    if not values:
        raise ValueError("Cannot calculate circular statistics from empty data.")

    angles = [value * pi / 180.0 for value in values]
    sin_mean = sum(sin(angle) for angle in angles) / len(angles)
    cos_mean = sum(cos(angle) for angle in angles) / len(angles)

    mean = degrees(atan2(sin_mean, cos_mean)) % 360.0

    resultant_length = sqrt(sin_mean ** 2 + cos_mean ** 2)

    if resultant_length <= 0.0:
        stddev = 180.0
    else:
        stddev = degrees(sqrt(-2.0 * log(resultant_length)))

    return Statistics(mean=mean, stddev=stddev)


def calculate_mode(values: Sequence[int]) -> int:
    if not values:
        raise ValueError("Cannot calculate mode from empty data.")

    return mode(values)