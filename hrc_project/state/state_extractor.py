"""Live pose extraction with independently testable proximity calculation."""

from datetime import datetime, timezone
import math

from .hrc_state import HRCState, Point2D


def validate_thresholds(caution_distance, stop_distance):
    if not all(math.isfinite(v) for v in (caution_distance, stop_distance)):
        raise ValueError("thresholds must be finite")
    if not 0 <= stop_distance <= caution_distance:
        raise ValueError("require 0 <= stop distance <= caution distance")


def calculate_state(*, simulation_step, robot_id, worker_id, robot_position,
                    worker_position, robot_direction, worker_direction,
                    robot_action, worker_action, task_phase,
                    caution_distance=300, stop_distance=150, timestamp=None):
    """Classify inclusive distance zones without selecting or blocking actions.

    Thresholds are provisional implementation parameters, not validated
    construction-safety distances. Stop-zone membership also implies caution.
    Inputs are any objects exposing finite x/y attributes.
    """
    validate_thresholds(caution_distance, stop_distance)
    points = []
    for point in (robot_position, worker_position, robot_direction, worker_direction):
        if not all(math.isfinite(v) for v in (point.x, point.y)):
            raise ValueError("pose coordinates must be finite")
        points.append(Point2D(float(point.x), float(point.y)))
    robot_position, worker_position, robot_direction, worker_direction = points
    distance = math.hypot(robot_position.x - worker_position.x,
                          robot_position.y - worker_position.y)
    return HRCState(
        timestamp=timestamp or datetime.now(timezone.utc).isoformat(),
        simulation_step=simulation_step, robot_id=robot_id, worker_id=worker_id,
        robot_position=robot_position, worker_position=worker_position,
        robot_direction=robot_direction, worker_direction=worker_direction,
        human_robot_distance=distance, robot_action=robot_action,
        worker_action=worker_action, task_phase=task_phase,
        worker_in_caution_zone=distance <= caution_distance,
        worker_in_stop_zone=distance <= stop_distance,
    )


class StateExtractor:
    """Read each actor's actual UE pose; never integrate actions to estimate it.

    Pose reads are sequential, not an atomic UE frame snapshot. Directions
    come from orientation[1] (yaw), matching the verified baseline.
    """

    def __init__(self, unrealcv, caution_distance=300, stop_distance=150):
        validate_thresholds(caution_distance, stop_distance)
        self.unrealcv = unrealcv
        self.caution_distance = caution_distance
        self.stop_distance = stop_distance

    def read_pose(self, actor_name):
        location = self.unrealcv.get_location(actor_name)
        orientation = self.unrealcv.get_orientation(actor_name)
        if len(location) != 3 or len(orientation) != 3:
            raise RuntimeError(f"Invalid UE pose response for {actor_name}")
        yaw = math.radians(float(orientation[1]))
        position = Point2D(float(location[0]), float(location[1]))
        direction = Point2D(math.cos(yaw), math.sin(yaw))
        if not all(math.isfinite(v) for v in (position.x, position.y, direction.x, direction.y)):
            raise RuntimeError(f"Non-finite UE pose response for {actor_name}")
        return {"position": position, "direction": direction}

    def extract(self, *, simulation_step, robot_id, worker_id, robot_name,
                worker_name, robot_action, worker_action, task_phase):
        robot, worker = self.read_pose(robot_name), self.read_pose(worker_name)
        return calculate_state(
            simulation_step=simulation_step, robot_id=robot_id, worker_id=worker_id,
            robot_position=robot["position"], worker_position=worker["position"],
            robot_direction=robot["direction"], worker_direction=worker["direction"],
            robot_action=robot_action, worker_action=worker_action,
            task_phase=task_phase, caution_distance=self.caution_distance,
            stop_distance=self.stop_distance,
        )
