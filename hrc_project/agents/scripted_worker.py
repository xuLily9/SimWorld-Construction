"""Deterministic delayed crossing, using supported fixed action commands."""

import math

from .scripted_agent import ScriptedAgent


class ScriptedWorker:
    """Wait for delay_steps, cross once, then wait on all subsequent steps.

    Delay is counted in scenario steps, not seconds. Spawn position and facing
    are configured by the runner. The default crossing is twelve half-second
    forward actions; custom crossing_actions may include verified rotations.
    No actions depend on robot proximity. reset() repeats the same sequence.
    """

    def __init__(self, delay_steps=2, crossing_actions=None,
                 crossing_steps=12, forward_duration=0.5):
        if isinstance(delay_steps, bool) or not isinstance(delay_steps, int) or delay_steps < 0:
            raise ValueError("delay_steps must be a non-negative integer")
        if not isinstance(crossing_steps, int) or crossing_steps <= 0:
            raise ValueError("crossing_steps must be a positive integer")
        if not math.isfinite(forward_duration) or forward_duration <= 0:
            raise ValueError("forward_duration must be finite and positive")
        actions = tuple(crossing_actions) if crossing_actions is not None else (
            f"forward {forward_duration:g}",
        ) * crossing_steps
        self.delay_steps = delay_steps
        self.crossing_actions = actions
        self.reset()

    def reset(self):
        self.policy = ScriptedAgent(self.crossing_actions)
        self.step_index = 0

    def action(self, obs=None, target=None):
        self.step_index += 1
        if self.step_index <= self.delay_steps:
            return "wait"
        return self.policy.action(obs, target) or "wait"

    @property
    def phase(self):
        if self.step_index <= self.delay_steps:
            return "worker_waiting"
        if self.step_index <= self.delay_steps + len(self.crossing_actions):
            return "worker_crossing"
        return "worker_finished"
