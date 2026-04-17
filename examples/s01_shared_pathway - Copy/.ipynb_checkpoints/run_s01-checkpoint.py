import os
import sys
import time

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))

if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from s01_env import S01Environment
from s01_agent import S01RuleAgent


def format_min_dist(value):
    if value == float("inf"):
        return "N/A"
    return f"{value:.1f}"


def summarize_nearby_agents(nearby_agents):
    if not nearby_agents:
        return "none"

    parts = []
    for agent in nearby_agents:
        parts.append(
            f"{agent['type']}:{agent['name']} dist={agent['distance']:.1f}"
        )
    return " | ".join(parts)


def run_episode(env, agent, episode_idx, max_steps=100, step_pause=0.1, verbose=True):
    obs = env.reset()

    total_reward = 0.0
    pickup_step = None
    dropoff_step = None
    stop_action_count = 0
    visible_pedestrian_available = getattr(env, "visible_pedestrian_available", False)

    if verbose:
        print(f"\n========== EPISODE {episode_idx} START ==========\n")
        print(
            f"[RUN] visible_pedestrian_available = {visible_pedestrian_available}"
        )

    step_num = 0
    info = {}

    for t in range(max_steps):
        step_num = t + 1

        if verbose:
            print(f"\n----- EPISODE {episode_idx} | STEP {step_num} -----")

        action = agent.act(obs)

        if action is not None and action.strip().lower() == "stop":
            stop_action_count += 1

        if verbose:
            print(f"[RUN] Action = {action}")

        obs, reward, done, info = env.step(action)
        total_reward += reward

        if pickup_step is None and info["carrying_material"]:
            pickup_step = step_num

        if info["completed"] and dropoff_step is None:
            dropoff_step = step_num

        if verbose:
            print(
                f"[RUN] reward={reward:.2f} "
                f"success={info['success']} "
                f"carrying={info['carrying_material']} "
                f"completed={info['completed']} "
                f"target={info['target_name']} "
                f"pickup_dist={info['pickup_dist']:.1f} "
                f"dropoff_dist={info['dropoff_dist']:.1f} "
                f"min_dynamic_distance={format_min_dist(info.get('min_dynamic_distance', float('inf')))} "
                f"min_ped_distance={format_min_dist(info.get('min_ped_distance', float('inf')))} "
                f"stop_count={info.get('stop_count', 0)} "
                f"near_collision_count={info.get('near_collision_count', 0)}"
            )

            nearby_summary = summarize_nearby_agents(info.get("nearby_agents", []))
            print(f"[RUN] nearby_agents = {nearby_summary}")

        if done:
            if verbose:
                print(f"\n[RUN] Episode {episode_idx} finished at step {step_num}.")
            break

        time.sleep(step_pause)

    episode_result = {
        "episode": episode_idx,
        "steps": step_num,
        "total_reward": total_reward,
        "completed": info.get("completed", False),
        "pickup_step": pickup_step,
        "dropoff_step": dropoff_step,
        "wait_count": info.get("wait_count", 0),
        "stop_count": info.get("stop_count", 0),
        "stop_action_count": stop_action_count,
        "near_collision_count": info.get("near_collision_count", 0),
        "final_pickup_dist": info.get("pickup_dist", float("inf")),
        "final_dropoff_dist": info.get("dropoff_dist", float("inf")),
        "min_dynamic_distance": info.get("min_dynamic_distance", float("inf")),
        "min_ped_distance": info.get("min_ped_distance", float("inf")),
        "min_forklift_distance": info.get("min_forklift_distance", float("inf")),
        "visible_pedestrian_available": info.get(
            "visible_pedestrian_available",
            visible_pedestrian_available,
        ),
    }

    print(
        f"\n[SUMMARY] Episode {episode_idx}: "
        f"completed={episode_result['completed']} | "
        f"visible_pedestrian={episode_result['visible_pedestrian_available']} | "
        f"steps={episode_result['steps']} | "
        f"total_reward={episode_result['total_reward']:.2f} | "
        f"pickup_step={episode_result['pickup_step']} | "
        f"dropoff_step={episode_result['dropoff_step']} | "
        f"stop_count={episode_result['stop_count']} | "
        f"stop_action_count={episode_result['stop_action_count']} | "
        f"near_collision_count={episode_result['near_collision_count']} | "
        f"min_dynamic_distance={format_min_dist(episode_result['min_dynamic_distance'])} | "
        f"min_ped_distance={format_min_dist(episode_result['min_ped_distance'])}"
    )

    return episode_result


def print_overall_summary(results):
    num_episodes = len(results)
    num_success = sum(1 for r in results if r["completed"])
    success_rate = num_success / num_episodes if num_episodes > 0 else 0.0

    avg_steps = (
        sum(r["steps"] for r in results) / num_episodes
        if num_episodes > 0 else 0.0
    )
    avg_reward = (
        sum(r["total_reward"] for r in results) / num_episodes
        if num_episodes > 0 else 0.0
    )
    avg_stop_count = (
        sum(r["stop_count"] for r in results) / num_episodes
        if num_episodes > 0 else 0.0
    )
    avg_stop_action_count = (
        sum(r["stop_action_count"] for r in results) / num_episodes
        if num_episodes > 0 else 0.0
    )
    avg_near_collision_count = (
        sum(r["near_collision_count"] for r in results) / num_episodes
        if num_episodes > 0 else 0.0
    )

    visible_success_count = sum(
        1 for r in results if r.get("visible_pedestrian_available", False)
    )

    successful_runs = [r for r in results if r["completed"]]
    if successful_runs:
        avg_success_steps = (
            sum(r["steps"] for r in successful_runs) / len(successful_runs)
        )
    else:
        avg_success_steps = None

    valid_min_dynamic = [
        r["min_dynamic_distance"]
        for r in results
        if r["min_dynamic_distance"] != float("inf")
    ]
    valid_min_ped = [
        r["min_ped_distance"]
        for r in results
        if r["min_ped_distance"] != float("inf")
    ]
    valid_min_forklift = [
        r["min_forklift_distance"]
        for r in results
        if r["min_forklift_distance"] != float("inf")
    ]

    overall_min_dynamic = min(valid_min_dynamic) if valid_min_dynamic else float("inf")
    overall_min_ped = min(valid_min_ped) if valid_min_ped else float("inf")
    overall_min_forklift = min(valid_min_forklift) if valid_min_forklift else float("inf")

    print("\n================ OVERALL SUMMARY ================")
    print(f"Episodes run                  : {num_episodes}")
    print(f"Successful episodes           : {num_success}")
    print(f"Success rate                  : {success_rate:.2%}")
    print(f"Visible pedestrian episodes   : {visible_success_count}/{num_episodes}")
    print(f"Average steps                 : {avg_steps:.2f}")
    print(f"Average total reward          : {avg_reward:.2f}")
    print(f"Average stop count            : {avg_stop_count:.2f}")
    print(f"Average stop action count     : {avg_stop_action_count:.2f}")
    print(f"Average near-collision count  : {avg_near_collision_count:.2f}")
    print(f"Overall min dynamic dist      : {format_min_dist(overall_min_dynamic)}")
    print(f"Overall min pedestrian dist   : {format_min_dist(overall_min_ped)}")
    print(f"Overall min forklift dist     : {format_min_dist(overall_min_forklift)}")

    if avg_success_steps is not None:
        print(f"Average steps (successful only): {avg_success_steps:.2f}")
    else:
        print("Average steps (successful only): N/A")

    print("=================================================\n")


def main():
    num_episodes = 2
    max_steps = 120
    step_pause = 0.05
    verbose = True

    env = S01Environment(
        roads_file="data/example_city/demo_city_1/roads.json"
    )
    agent = S01RuleAgent()

    results = []

    try:
        env.connect()

        print("\n[S01-B] Starting experiment...\n")

        for ep in range(1, num_episodes + 1):
            result = run_episode(
                env=env,
                agent=agent,
                episode_idx=ep,
                max_steps=max_steps,
                step_pause=step_pause,
                verbose=verbose,
            )
            results.append(result)

        print_overall_summary(results)

    finally:
        print("[S01-B] Disconnecting environment.")
        env.disconnect()


if __name__ == "__main__":
    main()