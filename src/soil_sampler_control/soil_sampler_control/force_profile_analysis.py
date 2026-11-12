from collections import deque


class MaxRateEstimator:

    def __init__(self, window: int, minimum_span: float) -> None:
        self.window = window
        self.minimum_span = minimum_span
        self.depths: deque[float] = deque(maxlen=window)
        self.forces: deque[float] = deque(maxlen=window)
        self.best_rate: float | None = None

    @property
    def max_rate(self) -> float:
        return self.best_rate if self.best_rate is not None else 0.0

    def update(self, depth: float, force: float) -> float | None:
        self.depths.append(depth)
        self.forces.append(force)

        if len(self.depths) < self.window:
            return None

        if max(self.depths) - min(self.depths) < self.minimum_span:
            return None

        count = len(self.depths)
        mean_depth = sum(self.depths) / count
        mean_force = sum(self.forces) / count

        denominator = sum((d - mean_depth) ** 2 for d in self.depths)

        if denominator <= 0.0:
            return None

        numerator = sum((d - mean_depth) * (f - mean_force) for d, f in zip(self.depths, self.forces))
        rate = numerator / denominator

        if self.best_rate is None or rate > self.best_rate:
            self.best_rate = rate

        return rate
