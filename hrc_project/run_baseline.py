"""Run fixed-action or direct goal-seeking navigation on the Base UE server."""

import argparse
import math
from pathlib import Path
import re
import socket
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from hrc_project.agents import DeterministicNavigator, ScriptedAgent
from hrc_project.logging_utils import ExperimentLogger


class Environment:
    """Notebook-compatible spawn, action execution, pose observation and reward.

    Camera acquisition is omitted because neither baseline uses images.
    action_success means a valid command returned without a Python exception;
    the Communicator does not propagate UE movement acknowledgement.
    """

    def __init__(self, communicator):
        # Import the existing client only for live runs, never for offline tests.
        from simworld.config import Config
        from simworld.map.map import Map
        from simworld.utils.vector import Vector

        self.Vector = Vector
        self.communicator = communicator
        self.config = Config()
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--agent", choices=("scripted", "deterministic"), default="scripted")
    parser.add_argument("--max-steps", type=positive_int, default=100)
    parser.add_argument("--repeat", action="store_true", help="repeat the scripted sequence")
    args = parser.parse_args(argv)
    policy = ScriptedAgent(repeat=args.repeat) if args.agent == "scripted" else DeterministicNavigator()
    logger = ExperimentLogger(args.agent)
    communicator = None
    ucv = None
    try:
        # Avoid the existing client's indefinite reconnect loop when UE is absent.
        try:
            with socket.create_connection(("127.0.0.1", 9000), timeout=3):
                pass
        except OSError as exc:
            raise RuntimeError("Start SimWorld.exe on the Base demo_1 map; port 9000 is unavailable.") from exc

        from simworld.communicator.communicator import Communicator
        from simworld.communicator.unrealcv import UnrealCV

        # Retain a reference for cleanup even if initialization fails part-way.
        ucv = UnrealCV.__new__(UnrealCV)
        ucv.__init__()
        communicator = Communicator(ucv)
        env = Environment(communicator)
        obs = env.reset()
        print(f"Agent: {args.agent}; target: {env.target}; starting position: {obs['position']}")
        for step in range(1, args.max_steps + 1):
            action = policy.action(obs, env.target)
            if action is None:
                print("Scripted sequence complete.")
                break
            obs, reward, success = env.step(action)
            entry = logger.record(step, obs, env.target, action, reward, success)
            print(f"Step {step}: {action}; position={obs['position']}; "
                  f"distance={entry['distance_to_target']:.2f}; reward={reward:.2f}; success={success}")
            if entry["distance_to_target"] <= 200:
                print("Target reached within 200 UE units.")
                break
        else:
            print("Maximum step count reached.")
    finally:
        try:
            if communicator is not None:
                communicator.disconnect()
            elif ucv is not None and hasattr(ucv, "client"):
                ucv.disconnect()
        except Exception as exc:
            print(f"Disconnect warning: {type(exc).__name__}", file=sys.stderr)
        finally:
            print(f"Log: {logger.path.resolve()}")


if __name__ == "__main__":
    main()
