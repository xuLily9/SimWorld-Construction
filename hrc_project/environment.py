"""Shared simulator environment and command-line validation for HRC runners."""

import argparse
import math
from pathlib import Path
import re
import time

REPO_ROOT = Path(__file__).resolve().parents[1]


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
        command = action.strip().lower()
        forward = re.fullmatch(r"forward\s+(\d+(?:\.\d+)?)", command)
        rotate = re.fullmatch(r"rotate\s+(\d+(?:\.\d+)?)\s+(left|right)", command)
        success = False
        if forward and float(forward[1]) > 0:
            self.communicator.humanoid_step_forward(self.agent.id, float(forward[1]), direction=0)
            success = True
        elif rotate and 0 < float(rotate[1]) <= 180:
            self.communicator.humanoid_rotate(self.agent.id, float(rotate[1]), rotate[2])
            success = True
        elif command == "wait":
            time.sleep(1)
            success = True
        obs = self.observe()
        reward = -obs["position"].distance(self.target)
        return obs, reward, success


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
