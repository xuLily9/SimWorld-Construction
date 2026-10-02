"""Two humanoid roles, sequential fixed actions, and live proximity observations."""

import argparse
from pathlib import Path
import socket
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from hrc_project.agents import DeterministicNavigator
from hrc_project.agents.scripted_worker import ScriptedWorker
from hrc_project.environment import nonnegative_int, positive_int
from hrc_project.logging_utils import HRCLogger
from hrc_project.environment import RoleEnvironment
from hrc_project.scenario import DEFAULT_CONFIG, ScenarioConfig, run_steps
from hrc_project.state import StateExtractor


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--max-steps", type=positive_int)
    parser.add_argument("--worker-delay-steps", type=nonnegative_int)
    parser.add_argument("--start-delay", type=nonnegative_int,
                        help="seconds to inspect the printed actor names before movement starts")
    parser.add_argument("--caution-distance", type=float)
    parser.add_argument("--stop-distance", type=float)
    args = parser.parse_args(argv)
    try:
        config = ScenarioConfig.load(args.config)
        data = config.data
        for override, section, key in (
            (args.max_steps, "run", "max_steps"), (args.worker_delay_steps, "worker", "delay_steps"),
            (args.start_delay, "run", "start_delay_seconds"), (args.caution_distance, "zones", "caution_distance"),
            (args.stop_distance, "zones", "stop_distance"),
        ):
            if override is not None:
                data[section][key] = override
        config = ScenarioConfig(data)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.error(str(exc))
    navigator = DeterministicNavigator(**config.navigator_settings())
    worker_policy = ScriptedWorker(**config.worker_settings())
    logger = HRCLogger(metadata={
        "robot_visual_placeholder": "humanoid", "config_path": str(args.config.resolve()),
        "scenario_config": config.data,
        "role_blueprints": {role: config.blueprint(role) for role in ("robot", "worker")},
        "world_geometry": {role: {key: vars(config.world_point(role, key)) for key in ("start", "target")}
                           for role in ("robot", "worker")},
        "thresholds_validated_for_safety": False, "execution_order": ["robot_role", "worker_role"],
    })
    communicator, ucv = None, None
    try:
        try:
            with socket.create_connection(("127.0.0.1", 9000), timeout=3):
                pass
        except OSError as exc:
            raise RuntimeError(f"Open {config.data['map']} and start the UnrealCV backend on port 9000.") from exc
        from simworld.communicator.communicator import Communicator
        from simworld.communicator.unrealcv import UnrealCV

        ucv = UnrealCV.__new__(UnrealCV)
        ucv.__init__()
        communicator = Communicator(ucv)
        old_names = [str(name) for name in ucv.get_objects() if str(name).startswith("GEN_BP_Humanoid_")]
        logger.metadata["preexisting_humanoids"] = {
            name: [float(value) for value in ucv.get_location(name)] for name in old_names
        }
        logger.flush()
        if old_names:
            print(f"[Existing actors] {', '.join(old_names)} remain from earlier runs. "
                  "They are not controlled or removed, and can block this trial. "
                  "Restart the backend before a comparable validation run.", flush=True)
        robot = RoleEnvironment(communicator, "robot", config)
        worker = RoleEnvironment(communicator, "worker", config)
        robot.reset()
        worker.reset()
        if robot.agent.id == worker.agent.id or robot.agent_name == worker.agent_name:
            raise RuntimeError("The backend did not create two distinct humanoids")
        logger.metadata["actors"] = {
            "robot_role": {"id": robot.agent.id, "name": robot.agent_name},
            "worker_role": {"id": worker.agent.id, "name": worker.agent_name},
        }
        print(f"robot_role (humanoid visual placeholder): {robot.agent_name}, id={robot.agent.id}")
        print(f"worker_role: {worker.agent_name}, id={worker.agent.id}")
        print("Only the actor names above belong to this run. Older actors are not controlled.", flush=True)
        logger.metadata["initial_poses"] = {
            role: {key: {"x": value.x, "y": value.y} for key, value in env.observe().items()}
            for role, env in (("robot", robot), ("worker", worker))
        }
        logger.flush()
        delay = config.data["run"]["start_delay_seconds"]
        if delay:
            print(f"Movement starts in {delay} seconds. Switch to the UE window now.", flush=True)
            time.sleep(delay)
        extractor = StateExtractor(ucv, **config.data["zones"])
        run_steps(robot, worker, navigator, worker_policy, extractor, logger, config.data["run"]["max_steps"], config)
    finally:
        try:
            if communicator is not None:
                communicator.disconnect()
            elif ucv is not None and hasattr(ucv, "client"):
                ucv.disconnect()
        except Exception as exc:
            print(f"Disconnect warning: {type(exc).__name__}", file=sys.stderr)
        finally:
            print(f"HRC log: {logger.path.resolve()}")


if __name__ == "__main__":
    main()
