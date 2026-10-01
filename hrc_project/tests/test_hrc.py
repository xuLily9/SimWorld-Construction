import json
import math
from types import SimpleNamespace

import pytest

from hrc_project.agents import DeterministicNavigator
from hrc_project.agents.scripted_worker import ScriptedWorker
from hrc_project.logging_utils import HRCLogger
from hrc_project.run_hrc_scenario import run_steps
from hrc_project.state import Point2D, StateExtractor, calculate_state


def state_at(distance, **kwargs):
    return calculate_state(
        simulation_step=1, robot_id=0, worker_id=1,
        robot_position=Point2D(0, 0), worker_position=Point2D(distance, 0),
        robot_direction=Point2D(1, 0), worker_direction=Point2D(0, 1),
        robot_action="forward 0.5", worker_action="wait", task_phase="worker_waiting",
        **kwargs,
    )


@pytest.mark.parametrize("distance,caution,stop", [
    (301, False, False), (300, True, False), (299, True, False),
    (151, True, False), (150, True, True), (149, True, True), (0, True, True),
])
def test_thresholds(distance, caution, stop):
    state = state_at(distance)
    assert state.human_robot_distance == distance
    assert state.worker_in_caution_zone is caution
    assert state.worker_in_stop_zone is stop


def test_distance_and_custom_thresholds():
    state = calculate_state(
        simulation_step=1, robot_id=4, worker_id=5,
        robot_position=Point2D(1, 2), worker_position=Point2D(4, 6),
        robot_direction=Point2D(1, 0), worker_direction=Point2D(0, 1),
        robot_action="wait", worker_action="wait", task_phase="worker_waiting",
        caution_distance=5, stop_distance=2,
    )
    assert state.human_robot_distance == 5
    assert state.worker_in_caution_zone
    assert not state.worker_in_stop_zone


@pytest.mark.parametrize("caution,stop", [(100, 150), (300, -1), (float("nan"), 150)])
def test_invalid_thresholds(caution, stop):
    with pytest.raises(ValueError):
        state_at(1, caution_distance=caution, stop_distance=stop)


def test_worker_order_reset_and_repeatability():
    first = ScriptedWorker(delay_steps=2, crossing_actions=["forward 1", "rotate 45 right", "forward 1"])
    second = ScriptedWorker(delay_steps=2, crossing_actions=["forward 1", "rotate 45 right", "forward 1"])
    expected = ["wait", "wait", "forward 1", "rotate 45 right", "forward 1", "wait", "wait"]
    assert [first.action() for _ in expected] == expected
    assert first.phase == "worker_finished"
    assert [second.action() for _ in expected] == expected
    first.reset()
    assert first.phase == "worker_waiting"
    assert [first.action() for _ in expected] == expected


def test_default_worker_delay_and_crossing():
    worker = ScriptedWorker()
    assert [worker.action() for _ in range(14)] == ["wait"] * 2 + ["forward 0.5"] * 12
    assert worker.action() == "wait"


def test_live_extractor_uses_actor_names_and_yaw():
    class Backend:
        def get_location(self, name):
            return [0, 0, 600] if name == "robot" else [3, 4, 600]

        def get_orientation(self, name):
            return [0, 0, 0] if name == "robot" else [0, 90, 0]

    state = StateExtractor(Backend()).extract(
        simulation_step=3, robot_id=4, worker_id=8, robot_name="robot", worker_name="worker",
        robot_action="forward 1", worker_action="wait", task_phase="worker_crossing",
    )
    assert state.human_robot_distance == 5
    assert state.worker_position == Point2D(3, 4)
    assert state.worker_direction.y == pytest.approx(1)
    assert state.worker_direction.x == pytest.approx(0, abs=1e-12)
    assert (state.robot_id, state.worker_id) == (4, 8)


def test_hrc_json_complete_and_unique(tmp_path):
    state = state_at(300)
    entry = json.loads(json.dumps(state.to_dict(), allow_nan=False))
    assert set(entry) == {
        "timestamp", "simulation_step", "robot_id", "worker_id", "robot_position",
        "worker_position", "robot_direction", "worker_direction", "human_robot_distance",
        "robot_action", "worker_action", "task_phase", "worker_in_caution_zone", "worker_in_stop_zone",
    }
    assert entry["robot_position"] == {"x": 0.0, "y": 0.0}
    logger = HRCLogger(tmp_path, metadata={"thresholds_validated_for_safety": False})
    logger.record(state)
    document = json.loads(logger.path.read_text())
    assert document["states"] == [entry]
    assert document["metadata"]["thresholds_validated_for_safety"] is False
    original = logger.path.read_text()
    other = HRCLogger(tmp_path)
    assert other.path != logger.path
    assert logger.path.read_text() == original


def test_nominal_crossing_trajectory_and_sequential_dispatch(tmp_path, capsys):
    """A kinematic fixture validates scenario geometry, not actual UE physics."""
    calls = []

    class Role:
        def __init__(self, name, id, position, direction, target):
            self.agent_name, self.agent = name, SimpleNamespace(id=id)
            self.position, self.direction, self.target = position, direction, target

        def observe(self):
            return {"position": self.position, "direction": self.direction}

        def step(self, action):
            calls.append(self.agent_name)
            if action.startswith("forward"):
                length = 200 * float(action.split()[1])
                self.position = Point2D(self.position.x + self.direction.x * length,
                                        self.position.y + self.direction.y * length)
            return self.observe(), 0, True

    robot = Role("robot", 0, Point2D(0, 0), Point2D(1, 0), Point2D(1600, 0))
    worker = Role("worker", 1, Point2D(600, -600), Point2D(0, 1), Point2D(600, 600))

    class Backend:
        def get_location(self, name):
            role = robot if name == "robot" else worker
            return [role.position.x, role.position.y, 600]

        def get_orientation(self, name):
            return [0, 0 if name == "robot" else 90, 0]

    logger = HRCLogger(tmp_path)
    events = run_steps(robot, worker, DeterministicNavigator(forward_duration=0.5),
                       ScriptedWorker(), StateExtractor(Backend()), logger, 20)
    assert events > 0
    assert calls == ["robot", "worker"] * 20
    assert worker.position.y > 0
    assert len({state["human_robot_distance"] for state in logger.entries}) > 1
    assert len(logger.entries) == 20
    assert any(state["worker_in_stop_zone"] for state in logger.entries)
    assert not logger.entries[0]["worker_in_caution_zone"]
    assert "Caution-zone samples:" in capsys.readouterr().out
