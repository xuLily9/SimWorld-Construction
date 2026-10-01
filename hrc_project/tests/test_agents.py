import math
from types import SimpleNamespace as Point

import pytest

from hrc_project.agents import DeterministicNavigator, ScriptedAgent


def observation(yaw=0):
    angle = math.radians(yaw)
    return {"position": Point(x=0, y=0), "direction": Point(x=math.cos(angle), y=math.sin(angle))}


def test_scripted_default_order_and_stop():
    agent = ScriptedAgent()
    assert [agent.action({}, None) for _ in range(5)] == list(ScriptedAgent.DEFAULT_ACTIONS)
    assert agent.action({}, None) is None
    assert agent.action({}, None) is None


def test_scripted_configured_repeat():
    actions = ["wait", "forward 2"]
    agent = ScriptedAgent(actions, repeat=True)
    actions.append("forward 3")
    assert [agent.action({}, None) for _ in range(5)] == ["wait", "forward 2", "wait", "forward 2", "wait"]


def test_scripted_configured_stop():
    agent = ScriptedAgent(["wait"])
    assert agent.action({}, None) == "wait"
    assert agent.action({}, None) is None


@pytest.mark.parametrize("target,expected", [
    (Point(x=1000, y=0), "forward 1"),
    (Point(x=0, y=-1000), "rotate 45 left"),
    (Point(x=0, y=1000), "rotate 45 right"),
    (Point(x=100, y=0), "wait"),
    (Point(x=200, y=0), "wait"),
])
def test_navigation(target, expected):
    assert DeterministicNavigator().action(observation(), target) == expected


@pytest.mark.parametrize("current,desired,turn", [(179, -179, "right"), (-179, 179, "left")])
def test_angle_wrap(current, desired, turn):
    target = Point(x=1000 * math.cos(math.radians(desired)), y=1000 * math.sin(math.radians(desired)))
    command = DeterministicNavigator(heading_tolerance_degrees=1).action(observation(current), target)
    verb, angle, direction = command.split()
    assert verb == "rotate"
    assert float(angle) == pytest.approx(2)
    assert direction == turn


def test_configurable_navigation():
    agent = DeterministicNavigator(goal_distance_threshold=1, heading_tolerance_degrees=10,
                                   maximum_turn_degrees=20, forward_duration=0.5)
    assert agent.action(observation(), Point(x=10, y=0)) == "forward 0.5"
    assert agent.action(observation(), Point(x=0, y=-10)) == "rotate 20 left"


@pytest.mark.parametrize("kwargs", [{"forward_duration": 0}, {"maximum_turn_degrees": 0},
                                    {"goal_distance_threshold": -1}, {"forward_duration": float("nan")}])
def test_invalid_settings(kwargs):
    with pytest.raises(ValueError):
        DeterministicNavigator(**kwargs)


@pytest.mark.parametrize("command,expected,success", [
    ("forward 0.5", ("forward", 7, 0.5, 0), True),
    ("rotate 45 left", ("rotate", 7, 45.0, "left"), True),
    ("rotate 45 right", ("rotate", 7, 45.0, "right"), True),
    ("forward 1 trailing", None, False),
    ("forward 0", None, False),
])
def test_environment_command_dispatch(command, expected, success):
    from hrc_project.run_baseline import Environment

    calls = []
    env = Environment.__new__(Environment)
    env.agent = Point(id=7)
    env.target = Point(x=0, y=0)
    position = Point(x=3, y=4, distance=lambda target: 5)
    env.observe = lambda: {"position": position, "direction": Point(x=1, y=0)}
    env.communicator = Point(
        humanoid_step_forward=lambda id, duration, direction: calls.append(("forward", id, duration, direction)),
        humanoid_rotate=lambda id, angle, direction: calls.append(("rotate", id, angle, direction)),
    )
    obs, reward, action_success = env.step(command)
    assert action_success is success
    assert reward == -5
    assert obs["position"] is position
    assert calls == ([] if expected is None else [expected])
