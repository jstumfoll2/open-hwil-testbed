"""Flight controllers that can sit in the loop.

Every controller takes the true state and the sensor frame each tick and
returns four normalized motor commands. The builtin controller uses the state;
PX4 (over MAVLink) only ever sees the sensor frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol

import numpy as np

from ..fdm import State
from ..mission import MissionSequencer
from ..sensors import BaroSample, GpsSample, ImuSample


@dataclass
class SensorFrame:
    t: float
    imu: ImuSample
    baro: BaroSample
    mag: np.ndarray
    gps: Optional[GpsSample]   # None on ticks without a new GPS sample


class Controller(Protocol):
    name: str
    mission: MissionSequencer

    def update(self, s: State, sensors: SensorFrame, dt: float) -> np.ndarray: ...

    def close(self) -> None: ...

    def info(self) -> dict: ...
