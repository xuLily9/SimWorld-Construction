"""Execute an HRC scenario and record observed movement and route crossing."""

import math

from hrc_project.scenario_config import ScenarioConfig
from hrc_project.state.crossing_tracker import CrossingTracker


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
