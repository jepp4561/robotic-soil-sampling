from dataclasses import dataclass
from enum import Enum, auto


class SamplerState(Enum):
    IDLE = auto()
    INSERTING = auto()
    ROCK_DETECTED = auto()
    DWELLING = auto()
    RETRACTING = auto()
    REPOSITIONING = auto()
    COMPLETE = auto()
    ERROR = auto()


@dataclass
class SamplerMeasurement:
    depth: float = 0.0
    force: float = 0.0
    vwc: float | None = None
    temperature: float | None = None
    ec: float | None = None


class SamplerStateMachine:

    def __init__(self, maximum_depth: float, maximum_force: float, maximum_attempts: int = 3) -> None:
        self.maximum_depth = maximum_depth
        self.maximum_force = maximum_force
        self.maximum_attempts = maximum_attempts

        self.state = SamplerState.IDLE
        self.measurement = SamplerMeasurement()
        self.insertion_attempt = 0

    def reset(self) -> None:
        self.state = SamplerState.IDLE
        self.measurement = SamplerMeasurement()
        self.insertion_attempt = 0

    def validate_target_depth(self, target_depth: float) -> tuple[bool, str]:
        if target_depth <= 0.0:
            return False, "Target depth must be greater than zero."

        if target_depth > self.maximum_depth:
            return (
                False,
                (f"Target depth {target_depth:.3f} mm exceeds " f"maximum depth {self.maximum_depth:.3f} m."),
            )

        return True, ""

    def update_actuator_measurement(self, depth: float, force: float) -> None:
        self.measurement.depth = depth
        self.measurement.force = force

    def update_soil_measurement(self, vwc: float | None = None, temperature: float | None = None, ec: float | None = None) -> None:
        if vwc is not None:
            self.measurement.vwc = vwc

        if temperature is not None:
            self.measurement.temperature = temperature

        if ec is not None:
            self.measurement.ec = ec

    def safety_check(self) -> tuple[bool, str]:
        if self.measurement.depth > self.maximum_depth:
            return (
                False,
                (f"Maximum depth exceeded: " f"{self.measurement.depth:.3f} > " f"{self.maximum_depth:.3f} m."),
            )

        if self.measurement.force > self.maximum_force:
            return (
                False,
                (f"Maximum force exceeded: " f"{self.measurement.force:.1f} > " f"{self.maximum_force:.1f} N."),
            )

        return True, ""

    def insertion_safety_check(self) -> tuple[bool, str]:
        if self.measurement.depth > self.maximum_depth:
            return (
                False,
                (f"Maximum depth exceeded: " f"{self.measurement.depth:.3f} > " f"{self.maximum_depth:.3f} m."),
            )

        if self.measurement.force > self.maximum_force:
            return (
                False,
                (f"Rock detected at " f"{self.measurement.depth:.3f} m: " f"force {self.measurement.force:.1f} > " f"{self.maximum_force:.1f} N."),
            )

        return True, ""

    def start_insertion(self) -> None:
        self.insertion_attempt += 1
        self.state = SamplerState.INSERTING

    def rock_detected(self) -> None:
        self.state = SamplerState.ROCK_DETECTED

    def start_dwell(self) -> None:
        self.state = SamplerState.DWELLING

    def start_retraction(self) -> None:
        self.state = SamplerState.RETRACTING

    def start_repositioning(self) -> None:
        self.state = SamplerState.REPOSITIONING

    def complete(self) -> None:
        self.state = SamplerState.COMPLETE

    def error(self) -> None:
        self.state = SamplerState.ERROR

    def can_retry(self) -> bool:
        return self.insertion_attempt < self.maximum_attempts

    def retry_count(self) -> int:
        return self.insertion_attempt

    def attempts_remaining(self) -> int:
        return max(0, self.maximum_attempts - self.insertion_attempt)
