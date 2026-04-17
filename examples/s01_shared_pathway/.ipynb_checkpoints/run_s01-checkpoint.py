import os
import sys

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))

if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from s01_env import S01Environment
from s01_agent import S01RuleAgent


def main():
    env = S01Environment(
        roads_file="data/example_city/demo_city_1/roads.json"
    )
    agent = S01RuleAgent()

    try:
        env.connect()
        obs = env.reset()

        print("\n[S01] Material Pickup Scenario Started\n")
        print("Goal: Robot picks up the box and places it in the drop zone.\n")

        for step_idx in range(100):
            print(f"===== STEP {step_idx + 1} =====")
            print(f"Robot position: {obs['robot_position']}")
            print(f"Material position: {obs['material_position']}")
            print(f"Drop zone position: {obs['drop_zone_position']}")
            print(f"Carrying: {obs['carrying_material']}")
            print(f"Picked: {obs['material_picked']}")
            print(f"Placed: {obs['material_placed']}")
            print(f"Distance to material: {obs['distance_to_material']}")
            print(f"Distance to drop zone: {obs['distance_to_drop_zone']}")

            action = agent.act(obs)
            print(f"Action = {action}")

            obs, reward, done, info = env.step(action)

            print(
                f"Reward={reward:.2f} | "
                f"Success={info['success']} | "
                f"Carrying={info['carrying_material']} | "
                f"Picked={info['material_picked']} | "
                f"Placed={info['material_placed']} | "
                f"Completed={info['task_completed']}"
            )
            print()

            if done:
                print("[S01] Episode finished.")
                break

    except KeyboardInterrupt:
        print("\n[S01] Interrupted by user.")
    finally:
        print("[S01] End.")


if __name__ == "__main__":
    main()