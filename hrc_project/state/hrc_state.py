"""Serializable state for a robot-role humanoid and a worker-role humanoid."""

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class HRCState:
    timestamp: str
    simulation_step: int
    robot_id: int
    worker_id: int
    robot_position: Point2D
    worker_position: Point2D
    robot_direction: Point2D
    worker_direction: Point2D
    human_robot_distance: float
    robot_action: str
    worker_action: str
    task_phase: str
    worker_in_caution_zone: bool
    worker_in_stop_zone: bool

    def to_dict(self):
        """Return the complete state, including nested x/y coordinates."""
        return asdict(self)
