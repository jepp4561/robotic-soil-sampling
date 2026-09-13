from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
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