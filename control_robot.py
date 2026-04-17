import time
import os
import keyboard

from simworld.config import Config
from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV
from simworld.map.map import Map
from simworld.agent.humanoid import Humanoid
from simworld.utils.vector import Vector

print("Connecting...")
communicator = Communicator(UnrealCV())
print("Connected!")

config = Config()
sim_map = Map(config)

roads_file = os.path.join("data", "example_city", "demo_city_1", "roads.json")
sim_map.initialize_map_from_file(roads_file=roads_file)

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

communicator.spawn_agent(
    agent,
    name=None,
    model_path=agent_bp,
    type="humanoid"
)

communicator.humanoid_set_speed(agent.id, 400)

print("机器人已生成。")
print("请把焦点切回 UE 窗口。")
print("控制键：I前进  K后退  J左转  L右转  Q退出")

running = True

def forward():
    communicator.humanoid_step_forward(agent.id, 1, direction=0)

def backward():
    communicator.humanoid_step_forward(agent.id, 1, direction=180)

def left():
    communicator.humanoid_rotate(agent.id, 10, "left")

def right():
    communicator.humanoid_rotate(agent.id, 10, "right")

def quit_program():
    global running
    running = False
    print("退出控制...")

# 全局热键：即使焦点在 UE，也能触发
keyboard.add_hotkey("i", forward, suppress=False)
keyboard.add_hotkey("k", backward, suppress=False)
keyboard.add_hotkey("j", left, suppress=False)
keyboard.add_hotkey("l", right, suppress=False)
keyboard.add_hotkey("q", quit_program, suppress=False)

try:
    while running:
        time.sleep(0.05)
finally:
    keyboard.unhook_all_hotkeys()
    communicator.disconnect()
    print("Disconnected.")
