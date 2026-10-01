"""A reproducible sequence of supported SimWorld actions."""


class ScriptedAgent:
    """Return one command per call; return None after a non-repeating sequence.

    Observations and targets are accepted for compatibility with the navigator.
    They do not influence this fixed-action policy.
    """

    DEFAULT_ACTIONS = (
        "forward 1", "forward 1", "rotate 45 right", "forward 1", "wait"
    )

    def __init__(self, actions=None, repeat=False):
        self.actions = tuple(self.DEFAULT_ACTIONS if actions is None else actions)
        if not self.actions or any(not isinstance(a, str) or not a.strip() for a in self.actions):
            raise ValueError("actions must contain at least one non-empty command")
        self.repeat = repeat
        self.step_index = 0

    def action(self, obs, target):
        if not self.repeat and self.step_index >= len(self.actions):
            return None
        command = self.actions[self.step_index % len(self.actions)]
        self.step_index += 1
        return command
