import math


class S01RuleAgent:
    def __init__(self):
        self.pickup_threshold = 150.0
        self.dropoff_threshold = 150.0

        # Navigation control
        self.rotate_threshold_deg = 12.0
        self.small_angle_forward_deg = 20.0
        self.min_rotate_step_deg = 5.0
        self.max_rotate_step_deg = 20.0
        self.forward_duration = 2.5

        # -----------------------------
        # S01-B safety parameters
        # Use hysteresis: stop threshold != resume threshold
        # -----------------------------
        self.pedestrian_stop_dist = 180.0
        self.pedestrian_resume_dist = 230.0

        self.forklift_stop_dist = 260.0
        self.forklift_resume_dist = 320.0

        self.safe_resume_steps = 1

        self.reset()

    def reset(self):
        """
        Reset episode-level memory.
        Call this once at the start of every episode.
        """
        self.safe_counter = 0
        self.stopped_for_safety = False
        self.last_blocking_type = None

    def _normalize_angle(self, angle_deg):
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg

    def _distance_xy(self, a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def _compute_rotate_step(self, angle_diff):
        """
        Adaptive turn step:
        large error -> larger turn
        small error -> smaller turn
        """
        abs_diff = abs(angle_diff)

        if abs_diff > 60:
            return self.max_rotate_step_deg
        if abs_diff > 30:
            return 12.0
        return self.min_rotate_step_deg

    def _check_dynamic_safety(self, obs):
        """
        Hysteresis safety logic:
        - while moving normally: use stop distance
        - once stopped for safety: use larger resume distance
        """
        position = obs["position"]
        nearby_agents = obs.get("nearby_agents", [])

        min_dist = None
        closest_type = None

        for agent in nearby_agents:
            agent_type = agent.get("type", "").lower()
            agent_pos = agent.get("position", None)

            if agent_pos is None:
                continue

            dist = self._distance_xy(position, agent_pos)

            if min_dist is None or dist < min_dist:
                min_dist = dist
                closest_type = agent_type

            if self.stopped_for_safety:
                if agent_type == "pedestrian" and dist < self.pedestrian_resume_dist:
                    self.last_blocking_type = agent_type
                    return True, f"pedestrian still too close for resume ({dist:.1f})"

                if agent_type == "forklift" and dist < self.forklift_resume_dist:
                    self.last_blocking_type = agent_type
                    return True, f"forklift still too close for resume ({dist:.1f})"
            else:
                if agent_type == "pedestrian" and dist < self.pedestrian_stop_dist:
                    self.last_blocking_type = agent_type
                    return True, f"pedestrian too close ({dist:.1f})"

                if agent_type == "forklift" and dist < self.forklift_stop_dist:
                    self.last_blocking_type = agent_type
                    return True, f"forklift too close ({dist:.1f})"

        if min_dist is not None:
            return False, f"closest={closest_type} dist={min_dist:.1f}"

        return False, "no nearby agents"

    def act(self, obs):
        position = obs["position"]
        direction = obs["direction"]
        carrying = obs["carrying_material"]

        target = obs["pickup_zone"] if not carrying else obs["dropoff_zone"]
        target_name = "pickup" if not carrying else "dropoff"

        # -----------------------------
        # Safety override
        # -----------------------------
        unsafe, safety_msg = self._check_dynamic_safety(obs)

        if unsafe:
            self.stopped_for_safety = True
            self.safe_counter = 0
            print(f"[AGENT] SAFETY STOP: {safety_msg}")
            return "stop"

        if self.stopped_for_safety:
            self.safe_counter += 1
            print(
                f"[AGENT] Safety cleared, resume check "
                f"({self.safe_counter}/{self.safe_resume_steps}) | {safety_msg}"
            )

            if self.safe_counter < self.safe_resume_steps:
                return "stop"

            print("[AGENT] Resuming task after clearance")
            self.stopped_for_safety = False
            self.safe_counter = 0
            self.last_blocking_type = None

        # -----------------------------
        # Task navigation logic
        # -----------------------------
        current_yaw = math.degrees(math.atan2(direction.y, direction.x))
        target_yaw = math.degrees(
            math.atan2(target.y - position.y, target.x - position.x)
        )
        angle_diff = self._normalize_angle(target_yaw - current_yaw)
        dist = position.distance(target)

        print(
            f"[AGENT] target={target_name} "
            f"dist={dist:.1f} "
            f"current_yaw={current_yaw:.1f} "
            f"target_yaw={target_yaw:.1f} "
            f"angle_diff={angle_diff:.1f}"
        )

        if not carrying and dist < self.pickup_threshold:
            print("[AGENT] Choosing: pickup")
            return "pickup"

        if carrying and dist < self.dropoff_threshold:
            print("[AGENT] Choosing: dropoff")
            return "dropoff"

        abs_diff = abs(angle_diff)

        if abs_diff <= self.rotate_threshold_deg:
            print(f"[AGENT] Choosing: forward {self.forward_duration:.1f}")
            return f"forward {self.forward_duration:.1f}"

        if abs_diff <= self.small_angle_forward_deg and dist > 250:
            print(
                f"[AGENT] Choosing: forward {self.forward_duration:.1f} (smooth mode)"
            )
            return f"forward {self.forward_duration:.1f}"

        rotate_step = self._compute_rotate_step(angle_diff)

        # In your SimWorld:
        # rotate right -> yaw increases
        # rotate left  -> yaw decreases
        if angle_diff > 0:
            print(f"[AGENT] Choosing: rotate {rotate_step:.0f} right")
            return f"rotate {rotate_step:.0f} right"

        print(f"[AGENT] Choosing: rotate {rotate_step:.0f} left")
        return f"rotate {rotate_step:.0f} left"