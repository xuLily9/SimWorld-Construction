"""Scenario configuration, coordinate transforms, execution and diagnostics."""

import copy
import json
import math
from pathlib import Path
import re

from hrc_project.agents import DeterministicNavigator
from hrc_project.agents.scripted_worker import ScriptedWorker
from hrc_project.state import Point2D
from hrc_project.state.crossing_tracker import CrossingTracker
from hrc_project.state.state_extractor import validate_thresholds

DEFAULT_CONFIG = Path(__file__).with_name("scenario.json")
VERIFIED_BLUEPRINT = "/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C"
PEDESTRIAN_BLUEPRINT = "/Game/TrafficSystem/Pedestrian/Base_Pedestrian.Base_Pedestrian_C"
SUPPORTED_BLUEPRINTS = {VERIFIED_BLUEPRINT, PEDESTRIAN_BLUEPRINT}


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
        if data["blueprint"] not in SUPPORTED_BLUEPRINTS:
            raise ValueError("blueprint must be Base_User_Agent or Base_Pedestrian")
        def check_numbers(value):
            if isinstance(value, dict):
                for item in value.values():
                    check_numbers(item)
            elif isinstance(value, (int, float)) and (isinstance(value, bool) or not math.isfinite(value)):
                raise ValueError("scenario numbers must be finite and not booleans")
        check_numbers(data)
        for name in ("robot", "worker"):
            if self.blueprint(name) not in SUPPORTED_BLUEPRINTS:
                raise ValueError(f"{name}.blueprint must be Base_User_Agent or Base_Pedestrian")
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

    def blueprint(self, role):
        """Select a role-specific model, falling back to the shared model."""
        return self.data[role].get("blueprint", self.data["blueprint"])

    def worker_settings(self):
        return {key: self.data["worker"][key] for key in ("delay_steps", "crossing_steps", "forward_duration")}


def run_steps(robot, worker, navigator, worker_policy, extractor, logger, max_steps, config=None):
    """Execute robot first and worker second, then sample both live poses.

    Each scenario step is two sequential blocking commands, not a synchronous
    UE physics tick. Proximity flags are observations only, never safety rules.
    """
    caution_events = 0
    config = config or ScenarioConfig.load()
    diagnostics = config.data["diagnostics"]
    tracker = CrossingTracker(config.world_point("robot", "start"), config.world_point("robot", "target"),
                              diagnostics["crossing_epsilon"])
    tracker.update(worker.observe()["position"])
    total_moved = {"robot": 0.0, "worker": 0.0}
    forward_counts = {"robot": 0, "worker": 0}
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
        displacement = {}
        for role, before, after, action, name in (
            ("robot", robot_obs["position"], state.robot_position, robot_action, robot.agent_name),
            ("worker", worker_obs["position"], state.worker_position, worker_action, worker.agent_name),
        ):
            moved = math.hypot(after.x - before.x, after.y - before.y)
            displacement[role] = moved
            total_moved[role] += moved
            forward_counts[role] += int(action.startswith("forward "))
            stationary_commands[role] = stationary_commands[role] + 1 if action.startswith("forward ") and moved < diagnostics["movement_epsilon"] else 0
            if stationary_commands[role] == diagnostics["stationary_forward_steps"]:
                print(f"[Movement warning] {role} actor={name}: {diagnostics['stationary_forward_steps']} forward commands but "
                      f"less than {diagnostics['movement_epsilon']} UE units of movement per step. Check UE pause/tick mode, "
                      "collision and the actor controller; command dispatch alone does not confirm movement.")
        crossing = tracker.update(state.worker_position)
        logger.record(state, {"robot_displacement": displacement["robot"],
                              "worker_displacement": displacement["worker"], **crossing})
        if crossing["worker_crossing_event"]:
            print(f"[Crossing observed] Step {step}: measured worker positions crossed the finite robot route.")
        caution_events += int(state.worker_in_caution_zone)
        print(f"Step {step}: robot={robot_action}; worker={worker_action}; "
              f"robot_position=({state.robot_position.x:.1f}, {state.robot_position.y:.1f}); "
              f"worker_position=({state.worker_position.x:.1f}, {state.worker_position.y:.1f}); "
              f"distance={state.human_robot_distance:.1f}; "
              f"caution={state.worker_in_caution_zone}; stop={state.worker_in_stop_zone}; "
              f"robot_moved={displacement['robot']:.1f}; worker_moved={displacement['worker']:.1f}; "
              f"crossed_route={tracker.crossed}")
    print(f"Caution-zone samples: {caution_events}")
    if not caution_events:
        print("No caution event observed: inspect live trajectories and spawn geometry before interpreting this run.")
    if not tracker.crossed:
        print("[Crossing missing] No measured side-to-side crossing within the finite robot route. "
              "Check worker heading, origin, obstacles, and crossing duration; completed script is not proof of crossing.")
    for role in total_moved:
        if total_moved[role] < diagnostics["movement_epsilon"]:
            print(f"[No movement] {role}: total displacement={total_moved[role]:.2f}, "
                  f"forward commands={forward_counts[role]}. Inspect pause/tick mode, controller and spawn/terrain.")
    robot_end = robot.observe()["position"]
    goal_distance = math.hypot(robot_end.x - robot.target.x, robot_end.y - robot.target.y)
    if goal_distance > navigator.goal_distance_threshold:
        print(f"[Goal not reached] robot remains {goal_distance:.1f} UE units from its target. "
              "Check actual heading, route clearance, speed and maximum steps.")
    logger.metadata["validation_summary"] = {
        "robot_total_displacement": total_moved["robot"], "worker_total_displacement": total_moved["worker"],
        "worker_crossed_route": tracker.crossed, "caution_samples": caution_events,
        "robot_distance_to_goal": goal_distance,
        "robot_reached_goal": goal_distance <= navigator.goal_distance_threshold,
        "scenario_validated": all(v >= diagnostics["movement_epsilon"] for v in total_moved.values())
                              and tracker.crossed and caution_events > 0 and goal_distance <= navigator.goal_distance_threshold,
    }
    logger.flush()
    print(f"Scenario validated: {logger.metadata['validation_summary']['scenario_validated']}")
    return caution_events
