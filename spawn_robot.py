import time
import math
import os

from simworld.config import Config
from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV
from simworld.map.map import Map
from simworld.agent.humanoid import Humanoid
from simworld.utils.vector import Vector

print("1) Connecting to SimWorld UE backend...")
communicator = Communicator(UnrealCV())
print("Connected.")

print("2) Loading config and map...")
config = Config()
sim_map = Map(config)

# demo_1 对应的 roads.json，官方最小示例就是这样初始化地图的
roads_file = os.path.join("data", "example_city", "demo_city_1", "roads.json")
sim_map.initialize_map_from_file(roads_file=roads_file)
print(f"Map loaded from: {roads_file}")

print("3) Preparing humanoid...")
spawn_location = Vector(0, 0)
spawn_forward = Vector(1, 0)

agent = Humanoid(
    communicator=communicator,
    position=spawn_location,
    direction=spawn_forward,
    config=config,
    map=sim_map
)

agent_bp = "/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C"

print("4) Spawning humanoid into UE world...")
communicator.spawn_agent(
    agent,
    name=None,
    model_path=agent_bp,
    type="humanoid"
)

communicator.humanoid_set_speed(agent.id, 200)
agent_name = communicator.get_humanoid_name(agent.id)

print("Spawned successfully.")
print("Agent ID:", agent.id)
print("Agent Name:", agent_name)

# 给它一点时间稳定出现
time.sleep(5)

print("5) Moving the humanoid a little so you can notice it...")
communicator.humanoid_step_forward(agent.id, 2, direction=0)
time.sleep(1)
communicator.humanoid_rotate(agent.id, 90, "left")
time.sleep(1)
communicator.humanoid_step_forward(agent.id, 2, direction=0)

print("Done. Check the UE window now.")
print("Press Ctrl+C to stop this script if needed.")
time.sleep(30)

communicator.disconnect()
print("Disconnected.")
