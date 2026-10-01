"""Two humanoid roles, sequential fixed actions, and live proximity observations."""

import argparse
import math
from pathlib import Path
import socket
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from hrc_project.agents import DeterministicNavigator
from hrc_project.agents.scripted_worker import ScriptedWorker
from hrc_project.logging_utils import HRCLogger
from hrc_project.run_baseline import Environment, positive_int
from hrc_project.state import Point2D, StateExtractor
from hrc_project.state.state_extractor import validate_thresholds


class RoleEnvironment(Environment):
    """Reuse the verified baseline's dispatch without changing baseline behavior.

    Humanoid IDs are allocated by SimWorld. Skip names already present in UE
    rather than reusing actors from a previous Python process. Camera IDs are
    allocated by Humanoid too, but are not used by this scenario.
    """

    def __init__(self, communicator, role, spawn_position, spawn_direction, target):
        super().__init__(communicator)
        self.role = role
        self.spawn_position = self.Vector(spawn_position.x, spawn_position.y)
        self.spawn_direction = self.Vector(spawn_direction.x, spawn_direction.y)
        self.target = self.Vector(target.x, target.y)

    def reset(self):
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
                model_path="/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C",
                type="humanoid",
            )
            time.sleep(1)
            if self.agent_name not in set(self.communicator.unrealcv.get_objects()):
                raise RuntimeError(f"Spawn not confirmed for {self.role}: {self.agent_name}")
        self.communicator.humanoid_stop(self.agent.id)
        self.communicator.unrealcv.set_location(
            [self.spawn_position.x, self.spawn_position.y, 600], self.agent_name,
        )
        yaw = math.degrees(math.atan2(self.spawn_direction.y, self.spawn_direction.x))
        self.communicator.unrealcv.set_orientation([0, yaw, 0], self.agent_name)
        self.communicator.humanoid_set_speed(self.agent.id, 200)
        time.sleep(5)
        return self.observe()


def nonnegative_int(value):
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return number


def run_steps(robot, worker, navigator, worker_policy, extractor, logger, max_steps):
    """Execute robot first and worker second, then sample both live poses.

    Each scenario step is two sequential blocking commands, not a synchronous
    UE physics tick. Proximity flags are observations only, never safety rules.
    """
    caution_events = 0
    stationary_commands = {"robot": 0, "worker": 0}
    for step in range(1, max_steps + 1):
        robot_obs = robot.observe()
        worker_obs = worker.observe()
        robot_action = navigator.action(robot_obs, robot.target)
        worker_action = worker_policy.action()
        _, _, robot_ok = robot.step(robot_action)
        _, _, worker_ok = worker.step(worker_action)
        if not robot_ok or not worker_ok:
            raise RuntimeError("Unsupported role action; scenario stopped")
        state = extractor.extract(
            simulation_step=step, robot_id=robot.agent.id, worker_id=worker.agent.id,
            robot_name=robot.agent_name, worker_name=worker.agent_name,
            robot_action=robot_action, worker_action=worker_action,
            task_phase=worker_policy.phase,
        )
        logger.record(state)
        displacement = {}
        for role, before, after, action, name in (
            ("robot", robot_obs["position"], state.robot_position, robot_action, robot.agent_name),
            ("worker", worker_obs["position"], state.worker_position, worker_action, worker.agent_name),
        ):
            moved = math.hypot(after.x - before.x, after.y - before.y)
            displacement[role] = moved
            stationary_commands[role] = stationary_commands[role] + 1 if action.startswith("forward ") and moved < 1 else 0
            if stationary_commands[role] == 3:
                print(f"[Movement warning] {role} actor={name}: three forward commands but "
                      "less than 1 UE unit of movement per step. Check UE pause/tick mode, "
                      "collision and the actor controller; command dispatch alone does not confirm movement.")
        caution_events += int(state.worker_in_caution_zone)
        print(f"Step {step}: robot={robot_action}; worker={worker_action}; "
              f"robot_position=({state.robot_position.x:.1f}, {state.robot_position.y:.1f}); "
              f"worker_position=({state.worker_position.x:.1f}, {state.worker_position.y:.1f}); "
              f"distance={state.human_robot_distance:.1f}; "
              f"caution={state.worker_in_caution_zone}; stop={state.worker_in_stop_zone}; "
              f"robot_moved={displacement['robot']:.1f}; worker_moved={displacement['worker']:.1f}")
    print(f"Caution-zone samples: {caution_events}")
    if not caution_events:
        print("No caution event observed: inspect live trajectories and spawn geometry before interpreting this run.")
    return caution_events


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-steps", type=positive_int, default=20)
    parser.add_argument("--worker-delay-steps", type=nonnegative_int, default=2)
    parser.add_argument("--start-delay", type=nonnegative_int, default=0,
                        help="seconds to inspect the printed actor names before movement starts")
    parser.add_argument("--caution-distance", type=float, default=300)
    parser.add_argument("--stop-distance", type=float, default=150)
    args = parser.parse_args(argv)
    try:
        validate_thresholds(args.caution_distance, args.stop_distance)
    except ValueError as exc:
        parser.error(str(exc))
    navigator = DeterministicNavigator(forward_duration=0.5)
    worker_policy = ScriptedWorker(delay_steps=args.worker_delay_steps)
    logger = HRCLogger(metadata={
        "robot_visual_placeholder": "humanoid", "map": "/Game/Maps/demo_1",
        "robot_spawn": {"x": 0, "y": 0}, "target": {"x": 1600, "y": 0},
        "worker_spawn": {"x": 600, "y": -600}, "speed": 200,
        "robot_forward_duration": 0.5, "worker_forward_duration": 0.5,
        "worker_crossing_steps": 12, "worker_delay_steps": args.worker_delay_steps,
        "caution_distance": args.caution_distance, "stop_distance": args.stop_distance,
        "thresholds_validated_for_safety": False, "execution_order": ["robot_role", "worker_role"],
    })
    communicator, ucv = None, None
    try:
        try:
            with socket.create_connection(("127.0.0.1", 9000), timeout=3):
                pass
        except OSError as exc:
            raise RuntimeError("Start the Base demo_1 SimWorld.exe backend on port 9000.") from exc
        from simworld.communicator.communicator import Communicator
        from simworld.communicator.unrealcv import UnrealCV

        ucv = UnrealCV.__new__(UnrealCV)
        ucv.__init__()
        communicator = Communicator(ucv)
        robot = RoleEnvironment(communicator, "robot_role", Point2D(0, 0),
                                Point2D(1, 0), Point2D(1600, 0))
        worker = RoleEnvironment(communicator, "worker_role", Point2D(600, -600),
                                 Point2D(0, 1), Point2D(600, 600))
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
        if args.start_delay:
            print(f"Movement starts in {args.start_delay} seconds. Switch to the UE window now.", flush=True)
            time.sleep(args.start_delay)
        extractor = StateExtractor(ucv, args.caution_distance, args.stop_distance)
        run_steps(robot, worker, navigator, worker_policy, extractor, logger, args.max_steps)
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
