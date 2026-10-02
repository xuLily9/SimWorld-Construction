"""Observed worker crossing of a finite robot route segment, independent of UE."""

import math


class CrossingTracker:
    """Require sampled poses on opposite sides, with an in-segment intersection.

    Touching the line does not count. Keep the last pose outside the epsilon
    band so a negative -> zero -> positive sequence is detected. A crossing of
    the infinite line beyond either route endpoint is rejected. Interpolation
    between observations is an estimate, not continuous trajectory perception.
    """

    def __init__(self, start, end, epsilon=1):
        self.start, self.end, self.epsilon = start, end, epsilon
        self.dx, self.dy = end.x - start.x, end.y - start.y
        self.length = math.hypot(self.dx, self.dy)
        if self.length <= 0 or epsilon <= 0:
            raise ValueError("nonzero route and positive epsilon required")
        self.previous = None
        self.crossed = False

    def side_distance(self, point):
        return (self.dx * (point.y - self.start.y) - self.dy * (point.x - self.start.x)) / self.length

    def update(self, point):
        side = self.side_distance(point)
        event = False
        if abs(side) > self.epsilon:
            if self.previous is not None:
                old_point, old_side = self.previous
                if old_side * side < 0:
                    fraction = old_side / (old_side - side)
                    x = old_point.x + fraction * (point.x - old_point.x)
                    y = old_point.y + fraction * (point.y - old_point.y)
                    projection = ((x - self.start.x) * self.dx + (y - self.start.y) * self.dy) / self.length ** 2
                    event = 0 <= projection <= 1
            self.previous = (point, side)
        self.crossed |= event
        return {"worker_route_signed_distance": side, "worker_crossing_event": event,
                "worker_crossed_route": self.crossed}
