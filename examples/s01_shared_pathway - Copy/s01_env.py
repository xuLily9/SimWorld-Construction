import os
import sys
import re
import time
import math

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))

if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from simworld.config import Config
from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV
from simworld.map.map import Map
from simworld.agent.humanoid import Humanoid
from simworld.utils.vector import Vector


class S01Environment:
    def __init__(self, roads_file: str, config=None):
        self.config = config or Config()
        self.communicator = Communicator(UnrealCV())

        self.map = Map(self.config)
        self.map.initialize_map_from_file(roads_file=roads_file)

        self.agent = None
        self.agent_name = None
        self.agent_spawned = False

        # -----------------------------
        # S01 scenario setup
        # -----------------------------
        self.spawn_location = Vector(0, 0)
        self.spawn_forward = Vector(1, 0)

        # Logical task zones
        self.pickup_zone = Vector(400, 0)
        self.dropoff_zone = Vector(1600, -400)

        # Slightly larger radii for easier debugging
        self.pickup_radius = 200.0
        self.dropoff_radius = 200.0

        # -----------------------------
        # S01-B dynamic safety setup
        # -----------------------------
        # First version: define logical moving agents here.
        # Later, you can replace these with real spawned SimWorld actors.
        self.dynamic_agents = [
            {
                "name": "ped_1",
                "type": "pedestrian",
                "position": Vector(900, 0),
                "velocity": Vector(0, 0),
            },
            {
                "name": "forklift_1",
                "type": "forklift",
                "position": Vector(1200, -150),
                "velocity": Vector(0, 0),
            },
        ]

        self.enable_pedestrian = True
        self.enable_forklift = False

        self.pedestrian_safe_dist = 200.0
        self.forklift_safe_dist = 300.0
        self.near_collision_dist = 120.0

        # Task state
        self.carrying_material = False
        self.completed = False

        # Metrics
        self.step_count = 0
        self.wait_count = 0
        self.stop_count = 0
        self.near_collision_count = 0
        self.min_dynamic_distance = float("inf")
        self.min_ped_distance = float("inf")
        self.min_forklift_distance = float("inf")

    def connect(self):
        # Kept for compatibility with run_s01.py
        return

    def disconnect(self):
        try:
            self.communicator.disconnect()
        except Exception as e:
            print(f"[DISCONNECT] Warning: {e}")

    def reset(self):
        agent_bp = "/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C"

        if not self.agent_spawned:
            self.agent = Humanoid(
                communicator=self.communicator,
                position=self.spawn_location,
                direction=self.spawn_forward,
                config=self.config,
                map=self.map,
            )

            self.communicator.spawn_agent(
                self.agent,
                name=None,
                model_path=agent_bp,
                type="humanoid",
            )

            self.communicator.humanoid_set_speed(self.agent.id, 180)
            self.agent_name = self.communicator.get_humanoid_name(self.agent.id)
            self.agent_spawned = True

            print(f"[RESET] Spawned agent: {self.agent_name}")

        else:
            location_3d = [self.spawn_location.x, self.spawn_location.y, 600]
            spawn_yaw = math.degrees(
                math.atan2(self.spawn_forward.y, self.spawn_forward.x)
            )
            orientation_3d = [0, spawn_yaw, 0]

            self.communicator.unrealcv.set_location(location_3d, self.agent_name)
            self.communicator.unrealcv.set_orientation(orientation_3d, self.agent_name)

            print(f"[RESET] Repositioned agent: {self.agent_name}")

        # Reset task state
        self.carrying_material = False
        self.completed = False

        # Reset metrics
        self.step_count = 0
        self.wait_count = 0
        self.stop_count = 0
        self.near_collision_count = 0
        self.min_dynamic_distance = float("inf")
        self.min_ped_distance = float("inf")
        self.min_forklift_distance = float("inf")

        # Reset logical dynamic agents
        self.dynamic_agents = [
            {
                "name": "ped_1",
                "type": "pedestrian",
                "position": Vector(900, 0),
                "velocity": Vector(0, -40),   # crossing motion
            },
            {
                "name": "forklift_1",
                "type": "forklift",
                "position": Vector(1200, -150),
                "velocity": Vector(-20, 0),
            },
        ]

        print("[RESET] S01 task reset")
        print(
            f"[RESET] Pickup zone  = ({self.pickup_zone.x:.1f}, {self.pickup_zone.y:.1f}), "
            f"radius={self.pickup_radius:.1f}"
        )
        print(
            f"[RESET] Dropoff zone = ({self.dropoff_zone.x:.1f}, {self.dropoff_zone.y:.1f}), "
            f"radius={self.dropoff_radius:.1f}"
        )
        print(
            f"[RESET] Dynamic agents enabled | "
            f"pedestrian={self.enable_pedestrian} forklift={self.enable_forklift}"
        )

        time.sleep(1.5)
        return self._get_observation()

    def _get_agent_position_and_direction(self):
        loc_3d = self.communicator.unrealcv.get_location(self.agent_name)
        position = Vector(loc_3d[0], loc_3d[1])

        orientation = self.communicator.unrealcv.get_orientation(self.agent_name)
        yaw = orientation[1]

        direction = Vector(
            math.cos(math.radians(yaw)),
            math.sin(math.radians(yaw))
        )

        return position, direction, yaw

    def _distance_xy(self, a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def _update_dynamic_agents(self):
        """
        First S01-B version:
        Move logical agents in 2D without spawning full SimWorld actors yet.
        This gives you a working shared-pathway scenario immediately.
        """
        for agent in self.dynamic_agents:
            agent_type = agent["type"]

            if agent_type == "pedestrian" and self.enable_pedestrian:
                agent["position"] = Vector(
                    agent["position"].x + agent["velocity"].x,
                    agent["position"].y + agent["velocity"].y
                )

                # Bounce within a small crossing corridor
                if agent["position"].y < -250 or agent["position"].y > 150:
                    agent["velocity"] = Vector(agent["velocity"].x, -agent["velocity"].y)

            elif agent_type == "forklift" and self.enable_forklift:
                agent["position"] = Vector(
                    agent["position"].x + agent["velocity"].x,
                    agent["position"].y + agent["velocity"].y
                )

                # Simple back-and-forth movement
                if agent["position"].x < 900 or agent["position"].x > 1400:
                    agent["velocity"] = Vector(-agent["velocity"].x, agent["velocity"].y)

    def _get_nearby_agents(self, robot_position):
        nearby_agents = []

        for agent in self.dynamic_agents:
            if agent["type"] == "pedestrian" and not self.enable_pedestrian:
                continue
            if agent["type"] == "forklift" and not self.enable_forklift:
                continue

            dist = self._distance_xy(robot_position, agent["position"])

            nearby_agents.append({
                "name": agent["name"],
                "type": agent["type"],
                "position": agent["position"],
                "velocity": agent["velocity"],
                "distance": dist,
            })

            self.min_dynamic_distance = min(self.min_dynamic_distance, dist)

            if agent["type"] == "pedestrian":
                self.min_ped_distance = min(self.min_ped_distance, dist)
            elif agent["type"] == "forklift":
                self.min_forklift_distance = min(self.min_forklift_distance, dist)

            if dist < self.near_collision_dist:
                self.near_collision_count += 1

        return nearby_agents

    def _get_observation(self):
        position, direction, yaw = self._get_agent_position_and_direction()

        ego_view = self.communicator.get_camera_observation(
            self.agent.camera_id,
            "lit"
        )

        pickup_dist = position.distance(self.pickup_zone)
        dropoff_dist = position.distance(self.dropoff_zone)

        nearby_agents = self._get_nearby_agents(position)

        min_agent_dist = float("inf")
        if nearby_agents:
            min_agent_dist = min(agent["distance"] for agent in nearby_agents)

        return {
            "position": position,
            "direction": direction,
            "yaw": yaw,
            "ego_view": ego_view,
            "pickup_zone": self.pickup_zone,
            "dropoff_zone": self.dropoff_zone,
            "pickup_dist": pickup_dist,
            "dropoff_dist": dropoff_dist,
            "carrying_material": self.carrying_material,
            "nearby_agents": nearby_agents,
            "min_dynamic_distance": min_agent_dist,
        }

    def _distance_to_current_target(self, obs):
        return obs["pickup_dist"] if not obs["carrying_material"] else obs["dropoff_dist"]

    def _current_target_name(self):
        return "pickup" if not self.carrying_material else "dropoff"

    def _execute_forward(self, duration: float) -> bool:
        print(f"[MOVE] forward duration={duration}")

        try:
            self.communicator.humanoid_step_forward(
                self.agent.id,
                duration,
                direction=0
            )
            return True
        except TypeError:
            try:
                self.communicator.humanoid_step_forward(
                    self.agent.id,
                    duration
                )
                return True
            except Exception as e:
                print(f"[MOVE] Forward failed: {e}")
                return False
        except Exception as e:
            print(f"[MOVE] Forward failed: {e}")
            return False

    def _execute_rotate(self, angle: float, turn_dir: str) -> bool:
        print(f"[MOVE] rotate angle={angle} dir={turn_dir}")

        try:
            self.communicator.humanoid_rotate(
                self.agent.id,
                angle,
                turn_dir
            )
            return True
        except Exception as e:
            print(f"[MOVE] Rotate failed: {e}")
            return False

    def _parse_and_execute(self, action: str):
        if action is None:
            print("[ACTION] Received None action")
            return False

        action_cleaned = action.strip().strip('"').strip("'").lower()
        success = False

        print(f"[ACTION] {action_cleaned}")

        if action_cleaned.startswith("forward"):
            match = re.search(r"forward\s+(\d+\.?\d*)", action_cleaned)
            if match:
                duration = float(match.group(1))
                success = self._execute_forward(duration)
            else:
                print("[ACTION] Could not parse forward action")

        elif action_cleaned.startswith("rotate"):
            match = re.search(r"rotate\s+(\d+\.?\d*)\s+(left|right)", action_cleaned)
            if match:
                angle = float(match.group(1))
                turn_dir = match.group(2)
                success = self._execute_rotate(angle, turn_dir)
            else:
                print("[ACTION] Could not parse rotate action")

        elif action_cleaned in ["wait", "stop"]:
            time.sleep(0.5)
            self.wait_count += 1
            if action_cleaned == "stop":
                self.stop_count += 1
            success = True

        elif action_cleaned == "pickup":
            obs = self._get_observation()
            dist = obs["pickup_dist"]

            print(f"[PICKUP] Distance to pickup zone = {dist:.2f}")

            if dist < self.pickup_radius and not self.carrying_material:
                self.carrying_material = True
                success = True
                print("[PICKUP] Success: now carrying material")
            else:
                print("[PICKUP] Failed: too far away or already carrying")

        elif action_cleaned == "dropoff":
            obs = self._get_observation()
            dist = obs["dropoff_dist"]

            print(f"[DROPOFF] Distance to dropoff zone = {dist:.2f}")

            if dist < self.dropoff_radius and self.carrying_material:
                self.carrying_material = False
                self.completed = True
                success = True
                print("[DROPOFF] Success: task completed")
            else:
                print("[DROPOFF] Failed: too far away or not carrying")

        else:
            print(f"[ACTION] Unsupported action: {action_cleaned}")

        return success

    def _compute_reward(self, obs, action_success):
        target_dist = self._distance_to_current_target(obs)

        reward = 0.0

        # Task shaping
        reward -= 0.01 * target_dist
        reward -= 0.1

        # Safety penalty
        reward -= 1.0 * self.near_collision_count

        # Encourage correct stopping near hazards
        if obs["min_dynamic_distance"] < float("inf"):
            if obs["min_dynamic_distance"] < self.pedestrian_safe_dist:
                reward -= 0.5

        # Small action failure penalty
        if not action_success:
            reward -= 0.5

        # Completion reward
        if self.completed:
            reward += 100.0

        return reward

    def step(self, action: str):
        self.step_count += 1

        # Update dynamic world first
        self._update_dynamic_agents()

        success = self._parse_and_execute(action)
        obs = self._get_observation()
        reward = self._compute_reward(obs, success)

        done = self.completed or self.step_count >= 100

        info = {
            "success": success,
            "completed": self.completed,
            "wait_count": self.wait_count,
            "stop_count": self.stop_count,
            "near_collision_count": self.near_collision_count,
            "carrying_material": self.carrying_material,
            "target_name": self._current_target_name(),
            "pickup_dist": obs["pickup_dist"],
            "dropoff_dist": obs["dropoff_dist"],
            "min_dynamic_distance": obs["min_dynamic_distance"],
            "min_ped_distance": self.min_ped_distance,
            "min_forklift_distance": self.min_forklift_distance,
            "nearby_agents": obs["nearby_agents"],
        }

        print(
            f"[STEP {self.step_count}] "
            f"pos=({obs['position'].x:.1f}, {obs['position'].y:.1f}) "
            f"yaw={obs['yaw']:.1f} "
            f"pickup_dist={obs['pickup_dist']:.1f} "
            f"dropoff_dist={obs['dropoff_dist']:.1f} "
            f"min_dynamic_distance={obs['min_dynamic_distance']:.1f} "
            f"carrying={self.carrying_material} "
            f"reward={reward:.2f} "
            f"done={done}"
        )

        for agent in obs["nearby_agents"]:
            print(
                f"    [DYNAMIC] {agent['type']} {agent['name']} "
                f"pos=({agent['position'].x:.1f}, {agent['position'].y:.1f}) "
                f"dist={agent['distance']:.1f}"
            )

        return obs, reward, done, info