"""Durable JSON step logs with unique, exclusively created run files."""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
from uuid import uuid4


class ExperimentLogger:
    """Store post-action observations; flush after each completed step.

    UTC timestamps identify sampling time, not simulation time. The run file is
    a JSON array, including an empty array when startup fails before any step.
    """

    def __init__(self, agent_type, log_dir=None):
        self.agent_type = agent_type
        self.run_id = uuid4().hex
        directory = Path(log_dir) if log_dir is not None else Path(__file__).parent / "logs"
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f"baseline_{self.run_id}.json"
        self.entries = []
        with self.path.open("x", encoding="utf-8") as stream:
            stream.write("[]\n")

    def record(self, step, obs, target, action, reward, action_success):
        position, direction = obs["position"], obs["direction"]
        entry = {
            "run_id": self.run_id,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "step": step,
            "agent_type": self.agent_type,
            "position": {"x": float(position.x), "y": float(position.y)},
            "direction": {"x": float(direction.x), "y": float(direction.y)},
            "action": action,
            "reward": float(reward),
            "action_success": bool(action_success),
            "distance_to_target": math.hypot(target.x - position.x, target.y - position.y),
        }
        json.dumps(entry, allow_nan=False)
        self.entries.append(entry)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.entries, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        temporary.replace(self.path)
        return entry
