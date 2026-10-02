"""Load and validate local scenario parameters and transform them to UE XY."""

import copy
import json
import math
from pathlib import Path
import re

from hrc_project.agents import DeterministicNavigator
from hrc_project.agents.scripted_worker import ScriptedWorker
from hrc_project.state import Point2D
from hrc_project.state.state_extractor import validate_thresholds

DEFAULT_CONFIG = Path(__file__).with_name("scenario.json")
VERIFIED_BLUEPRINT = "/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C"


class ScenarioConfig:
    """Local starts/targets are rotated and translated by the scenario origin.

    Spawn Z is an absolute UE height, not inferred terrain height. Map is
    descriptive: the user must open that map; this loader does not change UE maps.
    """

    def __init__(self, data):
        self.data = copy.deepcopy(data)
        map_path = self.data["map"]
        if not isinstance(map_path, str) or not re.fullmatch(r"/Game/(?:[A-Za-z0-9_]+/)*[A-Za-z0-9_]+(?:\.umap)?", map_path):
            raise ValueError("map must be a /Game/... level asset path")
        self.data["map"] = map_path.removesuffix(".umap")
        if data["blueprint"] != VERIFIED_BLUEPRINT:
            raise ValueError("The existing Base_User_Agent humanoid Blueprint is required")
        def check_numbers(value):
            if isinstance(value, dict):
                for item in value.values():
                    check_numbers(item)
            elif isinstance(value, (int, float)) and (isinstance(value, bool) or not math.isfinite(value)):
                raise ValueError("scenario numbers must be finite and not booleans")
        check_numbers(data)
        for name in ("robot", "worker"):
            for key in ("start", "target"):
                self.world_point(name, key)
            if data[name]["speed"] <= 0:
                raise ValueError("walking speed must be positive")
            self.direction(name)
        DeterministicNavigator(**self.navigator_settings())
        ScriptedWorker(**self.worker_settings())
        for section, key in (("run", "max_steps"), ("diagnostics", "stationary_forward_steps")):
            value = data[section][key]
            if not isinstance(value, int) or value <= 0:
                raise ValueError(f"{section}.{key} must be a positive integer")
        for key in ("start_delay_seconds", "spawn_settle_seconds"):
            if data["run"][key] < 0:
                raise ValueError(f"{key} must be non-negative")
        for key in ("movement_epsilon", "crossing_epsilon"):
            if data["diagnostics"][key] <= 0:
                raise ValueError(f"{key} must be positive")
        validate_thresholds(**data["zones"])
        start, target = self.world_point("robot", "start"), self.world_point("robot", "target")
        if math.hypot(target.x - start.x, target.y - start.y) <= 2 * data["diagnostics"]["crossing_epsilon"]:
            raise ValueError("robot route must be longer than twice the crossing tolerance")
        float(data["origin"]["spawn_z"])

    @classmethod
    def load(cls, path=DEFAULT_CONFIG):
        with Path(path).open(encoding="utf-8-sig") as stream:
            return cls(json.load(stream))

    def world_point(self, role, key):
        point, origin = self.data[role][key], self.data["origin"]
        yaw = math.radians(origin["yaw_degrees"])
        return Point2D(origin["x"] + point["x"] * math.cos(yaw) - point["y"] * math.sin(yaw),
                       origin["y"] + point["x"] * math.sin(yaw) + point["y"] * math.cos(yaw))

    def direction(self, role):
        yaw = math.radians(self.data[role]["heading_degrees"] + self.data["origin"]["yaw_degrees"])
        return Point2D(math.cos(yaw), math.sin(yaw))

    def navigator_settings(self):
        return {key: self.data["robot"][key] for key in (
            "goal_distance_threshold", "heading_tolerance_degrees", "maximum_turn_degrees", "forward_duration")}

    def worker_settings(self):
        return {key: self.data["worker"][key] for key in ("delay_steps", "crossing_steps", "forward_duration")}
