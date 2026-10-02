"""Run fixed-action or direct goal-seeking navigation on the Base UE server."""

import argparse
from pathlib import Path
import socket
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from hrc_project.agents import DeterministicNavigator, ScriptedAgent
from hrc_project.environment import Environment, positive_int
from hrc_project.logging_utils import ExperimentLogger


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
