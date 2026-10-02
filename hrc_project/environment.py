"""Shared simulator environment and command-line validation for HRC runners."""

import argparse
import math
from pathlib import Path
import re
import time

from hrc_project.scenario import PEDESTRIAN_BLUEPRINT

REPO_ROOT = Path(__file__).resolve().parents[1]


def parse_action(action):
    """Return (command, value, direction), or None for an unsupported action."""
    command = action.strip().lower()
    if command == "wait":
        return "wait", None, None
    forward = re.fullmatch(r"forward\s+(\d+(?:\.\d+)?)", command)
    if forward and float(forward[1]) > 0:
        return "forward", float(forward[1]), None
    rotate = re.fullmatch(r"rotate\s+(\d+(?:\.\d+)?)\s+(left|right)", command)
    if rotate and 0 < float(rotate[1]) <= 180:
        return "rotate", float(rotate[1]), rotate[2]
    return None


class Environment:
    """Notebook-compatible spawn, action execution, pose observation and reward.

    Camera acquisition is omitted because neither baseline uses images.
    action_success means a valid command returned without a Python exception;
    the Communicator does not propagate UE movement acknowledgement.
    """

    def __init__(self, communicator, *, load_demo_roads=True):
        # Import the existing client only for live runs, never for offline tests.
        from simworld.config import Config
        from simworld.utils.vector import Vector

        self.Vector = Vector
        self.communicator = communicator
        self.config = Config()
        self.map = None
        if load_demo_roads:
            from simworld.map.map import Map

            self.map = Map(self.config)
            self.map.initialize_map_from_file(
                roads_file=str(REPO_ROOT / "data/example_city/demo_city_1/roads.json")
            )
        self.agent = None
        self.agent_name = None
        self.target = Vector(1700, -1700)

    def observe(self):
        location = self.communicator.unrealcv.get_location(self.agent_name)
        orientation = self.communicator.unrealcv.get_orientation(self.agent_name)
        yaw = orientation[1]
        position = self.Vector(location[0], location[1])
        direction = self.Vector(math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
        self.agent.position = position
        self.agent.direction = yaw
        return {"position": position, "direction": direction}

    def reset(self):
        from simworld.agent.humanoid import Humanoid

        if self.agent is None:
            self.agent = Humanoid(
                communicator=self.communicator, position=self.Vector(0, 0),
                direction=self.Vector(1, 0), config=self.config, map=self.map,
            )
            self.communicator.spawn_agent(
                self.agent, name=None,
                model_path="/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C",
                type="humanoid",
            )
            self.agent_name = self.communicator.get_humanoid_name(self.agent.id)
            self.communicator.humanoid_set_speed(self.agent.id, 200)
        else:
            self.communicator.unrealcv.set_location([0, 0, 600], self.agent_name)
            self.communicator.unrealcv.set_orientation([0, 0, 0], self.agent_name)
        time.sleep(5)
        return self.observe()

    def step(self, action):
        parsed = parse_action(action)
        command, value, direction = parsed or (None, None, None)
        if command == "forward":
            self.communicator.humanoid_step_forward(self.agent.id, value, direction=0)
        elif command == "rotate":
            self.communicator.humanoid_rotate(self.agent.id, value, direction)
        elif command == "wait":
            time.sleep(1)
        obs = self.observe()
        reward = -obs["position"].distance(self.target)
        return obs, reward, parsed is not None


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def nonnegative_int(value):
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return number


class RoleEnvironment(Environment):
    """Spawn and reset a configured role using the shared action dispatcher.

    Humanoid IDs are allocated by SimWorld. Skip names already present in UE
    rather than reusing actors from a previous Python process. Camera IDs are
    allocated by Humanoid too, but are not used by this scenario.
    """

    def __init__(self, communicator, role, config):
        super().__init__(communicator, load_demo_roads=False)
        self.role = role
        self.scenario_config = config
        self.is_pedestrian = config.blueprint(role) == PEDESTRIAN_BLUEPRINT
        spawn_position, spawn_direction = config.world_point(role, "start"), config.direction(role)
        target = config.world_point(role, "target")
        self.spawn_position = self.Vector(spawn_position.x, spawn_position.y)
        self.spawn_direction = self.Vector(spawn_direction.x, spawn_direction.y)
        self.target = self.Vector(target.x, target.y)

    @property
    def spawn_location(self):
        return [self.spawn_position.x, self.spawn_position.y,
                self.scenario_config.data["origin"]["spawn_z"]]

    def _spawn(self):
        """Allocate an unused actor name and confirm the backend created it."""
        from simworld.agent.humanoid import Humanoid

        if self.agent is None:
            existing = set(self.communicator.unrealcv.get_objects())
            while True:
                candidate = Humanoid(
                    communicator=self.communicator, position=self.spawn_position,
                    direction=self.spawn_direction, config=self.config, map=self.map,
                )
                name = self.communicator.get_humanoid_name(candidate.id)
                if name not in existing:
                    break
            self.agent, self.agent_name = candidate, name
            self.communicator.spawn_agent(
                self.agent, name=None,
                model_path=self.scenario_config.blueprint(self.role),
                position=self.spawn_location,
                type="humanoid",
            )
            time.sleep(1)
            if self.agent_name not in set(self.communicator.unrealcv.get_objects()):
                raise RuntimeError(f"Spawn not confirmed for {self.role}: {self.agent_name}")

    def reset(self):
        self._spawn()
        if self.is_pedestrian:
            self.communicator.unrealcv.p_stop(self.agent_name)
        else:
            self.communicator.humanoid_stop(self.agent.id)
        self.communicator.unrealcv.set_location(
            self.spawn_location, self.agent_name,
        )
        yaw = math.degrees(math.atan2(self.spawn_direction.y, self.spawn_direction.x))
        self.communicator.unrealcv.set_orientation([0, yaw, 0], self.agent_name)
        speed = self.scenario_config.data[self.role]["speed"]
        if self.is_pedestrian:
            self.communicator.unrealcv.p_set_speed(self.agent_name, speed)
        else:
            self.communicator.humanoid_set_speed(self.agent.id, speed)
        time.sleep(self.scenario_config.data["run"]["spawn_settle_seconds"])
        return self.observe()

    def step(self, action):
        """Pedestrian templates use MoveForward/StopPedestrian, not StepForward."""
        if not self.is_pedestrian:
            return super().step(action)
        parsed = parse_action(action)
        command, value, direction = parsed or (None, None, None)
        backend = self.communicator.unrealcv
        if command == "forward":
            try:
                backend.p_move_forward(self.agent_name)
                time.sleep(value)
            finally:
                backend.p_stop(self.agent_name)
        elif command == "rotate":
            backend.p_stop(self.agent_name)
            backend.p_rotate(self.agent_name, value, direction)
            time.sleep(1)
        elif command == "wait":
            backend.p_stop(self.agent_name)
            time.sleep(1)
        obs = self.observe()
        position = obs["position"]
        reward = -math.hypot(position.x - self.target.x, position.y - self.target.y)
        return obs, reward, parsed is not None
