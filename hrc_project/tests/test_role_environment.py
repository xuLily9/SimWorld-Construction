from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from hrc_project.environment import RoleEnvironment


def pedestrian_env():
    env = RoleEnvironment.__new__(RoleEnvironment)
    env.is_pedestrian = True
    env.agent_name = "worker"
    env.target = SimpleNamespace(x=0, y=0)
    env.observe = lambda: {"position": SimpleNamespace(x=3, y=4)}
    env.communicator = SimpleNamespace(unrealcv=Mock())
    return env


@pytest.mark.parametrize("action,expected", [
    ("forward 0.5", ["p_move_forward", "p_stop"]),
    ("wait", ["p_stop"]),
    ("rotate 45 left", ["p_stop", "p_rotate"]),
    ("forward 0", []),
])
def test_pedestrian_commands(action, expected, monkeypatch):
    sleep = Mock()
    monkeypatch.setattr("hrc_project.environment.time.sleep", sleep)
    env = pedestrian_env()
    _, reward, success = env.step(action)
    backend = env.communicator.unrealcv
    assert [call[0] for call in backend.mock_calls] == expected
    assert success == bool(expected)
    assert reward == -5
    if action.startswith("forward") and success:
        backend.p_move_forward.assert_called_once_with("worker")
        backend.p_stop.assert_called_once_with("worker")
        sleep.assert_called_once_with(0.5)
    if action.startswith("rotate"):
        backend.p_rotate.assert_called_once_with("worker", 45, "left")


def test_pedestrian_stops_when_forward_interrupted(monkeypatch):
    monkeypatch.setattr("hrc_project.environment.time.sleep", Mock(side_effect=KeyboardInterrupt))
    env = pedestrian_env()
    with pytest.raises(KeyboardInterrupt):
        env.step("forward 0.5")
    env.communicator.unrealcv.p_stop.assert_called_once_with("worker")


def test_robot_keeps_humanoid_dispatch(monkeypatch):
    env = pedestrian_env()
    env.is_pedestrian = False
    dispatch = Mock(return_value=({}, 0, True))
    monkeypatch.setattr("hrc_project.environment.Environment.step", dispatch)
    assert env.step("forward 0.5") == ({}, 0, True)
    dispatch.assert_called_once_with("forward 0.5")
    assert not env.communicator.unrealcv.mock_calls
