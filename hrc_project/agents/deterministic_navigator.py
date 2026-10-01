"""Direct goal seeking; this policy does not perform obstacle avoidance."""

import math


class DeterministicNavigator:
    """Turn toward a goal, then move for a fixed duration, or wait at the goal.

    Positions and directions must expose x/y attributes, like SimWorld Vector.
    Heading error is normalized to [-180, 180). Unreal yaw increases toward +Y:
    the inspected UnrealCV wrapper uses positive angles for right turns and
    negative angles for left turns. Thus positive heading error selects right.
    An exactly opposite target deterministically selects left.
    No perception, collision reasoning, or path planning is performed.
    """

    def __init__(self, goal_distance_threshold=200, heading_tolerance_degrees=5,
                 maximum_turn_degrees=45, forward_duration=1):
        values = (goal_distance_threshold, heading_tolerance_degrees,
                  maximum_turn_degrees, forward_duration)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("navigation settings must be finite")
        if goal_distance_threshold < 0 or not 0 <= heading_tolerance_degrees < 180:
            raise ValueError("invalid goal threshold or heading tolerance")
        if not 0 < maximum_turn_degrees <= 180 or forward_duration <= 0:
            raise ValueError("turn must be in (0, 180] and duration must be positive")
        self.goal_distance_threshold = goal_distance_threshold
        self.heading_tolerance_degrees = heading_tolerance_degrees
        self.maximum_turn_degrees = maximum_turn_degrees
        self.forward_duration = forward_duration

    def action(self, obs, target):
        position, direction = obs["position"], obs["direction"]
        dx, dy = target.x - position.x, target.y - position.y
        if math.hypot(dx, dy) <= self.goal_distance_threshold:
            return "wait"
        if not all(math.isfinite(v) for v in (dx, dy, direction.x, direction.y)):
            raise ValueError("observation and target coordinates must be finite")
        if math.hypot(direction.x, direction.y) == 0:
            raise ValueError("direction must be nonzero")
        desired = math.degrees(math.atan2(dy, dx))
        current = math.degrees(math.atan2(direction.y, direction.x))
        error = (desired - current + 180) % 360 - 180
        if abs(error) > self.heading_tolerance_degrees:
            angle = min(abs(error), self.maximum_turn_degrees)
            return f"rotate {angle:g} {'right' if error > 0 else 'left'}"
        return f"forward {self.forward_duration:g}"
