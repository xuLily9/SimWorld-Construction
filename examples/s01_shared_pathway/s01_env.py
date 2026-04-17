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
    """
    Scenario 01:
    Robot goes to pickup zone -> picks up visible box -> carries it ->
    goes to dropoff zone -> drops it off.

    Also includes one simple moving pedestrian for shared-path safety.
    """

    def __init__(self, roads_file: str, config=None):
        self.config = config or Config()
        self.communicator = Communicator(UnrealCV())

        self.map = Map(self.config)
        self.map.initialize_map_from_file(roads_file=roads_file)

        # --------------------------------------------------
        # Robot state
        # --------------------------------------------------
        self.agent = None
        self.agent_name = None
        self.agent_spawned = False

        self.spawn_location = Vector(0, 0)
        self.spawn_forward = Vector(1, 0)

        # --------------------------------------------------
        # Task setup
        # --------------------------------------------------
        self.pickup_zone = Vector(400, 0)
        self.dropoff_zone = Vector(1600, -400)

        self.pickup_radius = 200.0
        self.dropoff_radius = 200.0

        self.carrying_material = False
        self.completed = False

        # --------------------------------------------------
        # Visible task objects
        # --------------------------------------------------
        self.material_name = "material_box"
        self.drop_zone_name = "drop_zone_marker"

        self.material_asset_candidates = [
            "/Game/CityDatabase/blueprints/BP_Box.BP_Box_C",
            "/Game/CityDatabase/blueprints/BP_Box2.BP_Box2_C",
            "/Game/CityDatabase/blueprints/BP_Box3.BP_Box3_C",
        ]

        self.material_location = Vector(self.pickup_zone.x, self.pickup_zone.y)
        self.drop_zone_location = Vector(self.dropoff_zone.x, self.dropoff_zone.y)

        self.material_spawned = False
        self.drop_zone_spawned = False

        self.object_z = 30
        self.carried_box_z = 95

        # --------------------------------------------------
        # Simple pedestrian safety setup
        # --------------------------------------------------
        self.enable_pedestrian = True
        self.pedestrian_safe_dist = 200.0
        self.near_collision_dist = 120.0

        self.dynamic_agents = []
        self._reset_dynamic_agents()

        # --------------------------------------------------
        # Metrics
        # --------------------------------------------------
        self.step_count = 0
        self.wait_count = 0
        self.stop_count = 0
        self.near_collision_count = 0
        self.min_dynamic_distance = float("inf")
        self.min_ped_distance = float("inf")

    # ==================================================
    # Basic lifecycle
    # ==================================================

    def connect(self):
        return

    def disconnect(self):
        try:
            self.communicator.disconnect()
        except Exception as e:
            print(f"[DISCONNECT] Warning: {e}")

    def reset(self):
        self._spawn_or_reset_agent()
        self._reset_task_state()
        self._reset_metrics()
        self._reset_dynamic_agents()
        self._spawn_or_reset_task_objects()
        self._print_reset_info()

        time.sleep(1.5)
        return self._get_observation()

    # ==================================================
    # Reset helpers
    # ==================================================

    def _spawn_or_reset_agent(self):
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

    def _reset_task_state(self):
        self.carrying_material = False
        self.completed = False
        self.material_location = Vector(self.pickup_zone.x, self.pickup_zone.y)

    def _reset_metrics(self):
        self.step_count = 0
        self.wait_count = 0
        self.stop_count = 0
        self.near_collision_count = 0
        self.min_dynamic_distance = float("inf")
        self.min_ped_distance = float("inf")

    def _reset_dynamic_agents(self):
        self.dynamic_agents = [
            {
                "name": "ped_1",
                "type": "pedestrian",
                "position": Vector(900, 0),
                "velocity": Vector(0, -40),
            }
        ]

    def _print_reset_info(self):
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
            f"[RESET] Material box = ({self.material_location.x:.1f}, {self.material_location.y:.1f})"
        )
        print(f"[RESET] Pedestrian enabled = {self.enable_pedestrian}")

    # ==================================================
    # Visual task objects
    # ==================================================

    def _spawn_or_reset_task_objects(self):
        material_asset = self.material_asset_candidates[0]
        drop_zone_asset = self.material_asset_candidates[1]

        if not self.material_spawned:
            ok = self._spawn_static_object(
                object_name=self.material_name,
                asset_path=material_asset,
                position=self.material_location,
                z=self.object_z,
            )
            self.material_spawned = ok
            print(f"[RESET] Material spawned: {ok}")
        else:
            self._set_object_location(
                self.material_name,
                self.material_location,
                z=self.object_z
            )
            print("[RESET] Material repositioned")

        if not self.drop_zone_spawned:
            ok = self._spawn_static_object(
                object_name=self.drop_zone_name,
                asset_path=drop_zone_asset,
                position=self.drop_zone_location,
                z=self.object_z,
            )
            self.drop_zone_spawned = ok
            print(f"[RESET] Drop zone marker spawned: {ok}")
        else:
            self._set_object_location(
                self.drop_zone_name,
                self.drop_zone_location,
                z=self.object_z
            )
            print("[RESET] Drop zone marker repositioned")

    def _spawn_static_object(self, object_name, asset_path, position, z=30):
        location_3d = [position.x, position.y, z]

        try:
            self.communicator.unrealcv.spawn_object(
                object_name,
                asset_path,
                location_3d
            )
            return True
        except Exception as e1:
            try:
                self.communicator.spawn_object(
                    object_name,
                    asset_path,
                    location_3d
                )
                return True
            except Exception as e2:
                print(f"[SPAWN] Failed for {object_name}: {e1} | {e2}")
                return False

    def _set_object_location(self, object_name, position, z=30):
        location_3d = [position.x, position.y, z]

        try:
            self.communicator.unrealcv.set_location(location_3d, object_name)
            return True
        except Exception as e1:
            try:
                self.communicator.set_object_location(object_name, location_3d)
                return True
            except Exception as e2:
                print(f"[SET_OBJECT] Failed for {object_name}: {e1} | {e2}")
                return False

    def _update_carried_material_visual(self):
        if not self.carrying_material:
            return

        robot_position, _, _ = self._get_agent_position_and_direction()

        carried_pos = Vector(robot_position.x + 30, robot_position.y)
        self.material_location = carried_pos

        self._set_object_location(
            self.material_name,
            self.material_location,
            z=self.carried_box_z
        )

    # ==================================================
    # Robot state
    # ==================================================

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

    # ==================================================
    # Dynamic pedestrian
    # ==================================================

    def _update_dynamic_agents(self):
        for agent in self.dynamic_agents:
            if agent["type"] == "pedestrian" and self.enable_pedestrian:
                self._move_pedestrian(agent)

    def _move_pedestrian(self, agent):
        agent["position"] = Vector(
            agent["position"].x + agent["velocity"].x,
            agent["position"].y + agent["velocity"].y
        )

        if agent["position"].y < -250 or agent["position"].y > 150:
            agent["velocity"] = Vector(agent["velocity"].x, -agent["velocity"].y)

    def _get_nearby_agents(self, robot_position):
        nearby_agents = []

        for agent in self.dynamic_agents:
            if agent["type"] == "pedestrian" and not self.enable_pedestrian:
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
            self.min_ped_distance = min(self.min_ped_distance, dist)

            if dist < self.near_collision_dist:
                self.near_collision_count += 1

        return nearby_agents

    # ==================================================
    # Observation
    # ==================================================

    def _get_observation(self):
        position, direction, yaw = self._get_agent_position_and_direction()

        ego_view = self.communicator.get_camera_observation(
            self.agent.camera_id,
            "lit"
        )

        pickup_dist = position.distance(self.pickup_zone)
        dropoff_dist = position.distance(self.dropoff_zone)
        material_dist = position.distance(self.material_location)

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
            "material_location": self.material_location,
            "pickup_dist": pickup_dist,
            "dropoff_dist": dropoff_dist,
            "material_dist": material_dist,
            "carrying_material": self.carrying_material,
            "completed": self.completed,
            "nearby_agents": nearby_agents,
            "min_dynamic_distance": min_agent_dist,
        }

    def _current_target_name(self):
        return "pickup" if not self.carrying_material else "dropoff"

    def _distance_to_current_target(self, obs):
        if not obs["carrying_material"]:
            return obs["pickup_dist"]
        return obs["dropoff_dist"]

    # ==================================================
    # Action execution
    # ==================================================

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
                self.communicator.humanoid_step_forward(self.agent.id, duration)
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

    def _execute_pickup(self) -> bool:
        obs = self._get_observation()
        dist = obs["pickup_dist"]

        print(f"[PICKUP] Distance to pickup zone = {dist:.2f}")

        if self.carrying_material:
            print("[PICKUP] Failed: already carrying material")
            return False

        if dist >= self.pickup_radius:
            print("[PICKUP] Failed: too far from pickup zone")
            return False

        self.carrying_material = True
        self._update_carried_material_visual()

        print("[PICKUP] Success: now carrying material")
        return True

    def _execute_dropoff(self) -> bool:
        obs = self._get_observation()
        dist = obs["dropoff_dist"]

        print(f"[DROPOFF] Distance to dropoff zone = {dist:.2f}")

        if not self.carrying_material:
            print("[DROPOFF] Failed: not carrying material")
            return False

        if dist >= self.dropoff_radius:
            print("[DROPOFF] Failed: too far from dropoff zone")
            return False

        self.carrying_material = False
        self.completed = True

        self.material_location = Vector(
            self.dropoff_zone.x,
            self.dropoff_zone.y
        )

        self._set_object_location(
            self.material_name,
            self.material_location,
            z=self.object_z
        )

        print("[DROPOFF] Success: task completed")
        return True

    def _parse_and_execute(self, action: str):
        if action is None:
            print("[ACTION] Received None action")
            return False

        action_cleaned = action.strip().strip('"').strip("'").lower()
        print(f"[ACTION] {action_cleaned}")

        if action_cleaned.startswith("forward"):
            match = re.search(r"forward\s+(\d+\.?\d*)", action_cleaned)
            if match:
                duration = float(match.group(1))
                return self._execute_forward(duration)

            print("[ACTION] Could not parse forward action")
            return False

        if action_cleaned.startswith("rotate"):
            match = re.search(r"rotate\s+(\d+\.?\d*)\s+(left|right)", action_cleaned)
            if match:
                angle = float(match.group(1))
                turn_dir = match.group(2)
                return self._execute_rotate(angle, turn_dir)

            print("[ACTION] Could not parse rotate action")
            return False

        if action_cleaned in ["wait", "stop"]:
            time.sleep(0.5)
            self.wait_count += 1
            if action_cleaned == "stop":
                self.stop_count += 1
            return True

        if action_cleaned == "pickup":
            return self._execute_pickup()

        if action_cleaned == "dropoff":
            return self._execute_dropoff()

        print(f"[ACTION] Unsupported action: {action_cleaned}")
        return False

    # ==================================================
    # Reward
    # ==================================================

    def _compute_reward(self, obs, action_success):
        target_dist = self._distance_to_current_target(obs)

        reward = 0.0
        reward -= 0.01 * target_dist
        reward -= 0.1

        reward -= 1.0 * self.near_collision_count

        if obs["min_dynamic_distance"] < float("inf"):
            if obs["min_dynamic_distance"] < self.pedestrian_safe_dist:
                reward -= 0.5

        if not action_success:
            reward -= 0.5

        if self.completed:
            reward += 100.0

        return reward

    # ==================================================
    # Step
    # ==================================================

    def step(self, action: str):
        self.step_count += 1

        self._update_dynamic_agents()
        success = self._parse_and_execute(action)
        self._update_carried_material_visual()

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
            "material_dist": obs["material_dist"],
            "material_location": obs["material_location"],
            "min_dynamic_distance": obs["min_dynamic_distance"],
            "min_ped_distance": self.min_ped_distance,
            "nearby_agents": obs["nearby_agents"],
        }

        print(
            f"[STEP {self.step_count}] "
            f"pos=({obs['position'].x:.1f}, {obs['position'].y:.1f}) "
            f"yaw={obs['yaw']:.1f} "
            f"pickup_dist={obs['pickup_dist']:.1f} "
            f"dropoff_dist={obs['dropoff_dist']:.1f} "
            f"material=({obs['material_location'].x:.1f}, {obs['material_location'].y:.1f}) "
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