import json

import pytest

from hrc_project.scenario import ScenarioConfig
from hrc_project.state import Point2D
from hrc_project.state.crossing_tracker import CrossingTracker


@pytest.mark.parametrize("map_path", ["/Game/Maps/demo_1", "/Game/HRConstruction/Model", "/Game/HRConstruction/Model.umap"])
def test_custom_map_path(map_path):
    data = ScenarioConfig.load().data
    data["map"] = map_path
    assert ScenarioConfig(data).data["map"] == map_path.removesuffix(".umap")


@pytest.mark.parametrize("map_path", ["", "Model.umap", "/Game/../Model", None])
def test_invalid_map_path(map_path):
    data = ScenarioConfig.load().data
    data["map"] = map_path
    with pytest.raises(ValueError, match="level asset path"):
        ScenarioConfig(data)


def test_origin_translation_and_rotation():
    data = ScenarioConfig.load().data
    data["origin"].update(x=1000, y=2000, yaw_degrees=90)
    config = ScenarioConfig(data)
    assert config.world_point("robot", "start") == Point2D(1000, 2000)
    end = config.world_point("robot", "target")
    assert end.x == pytest.approx(1000)
    assert end.y == pytest.approx(3600)
    start = config.world_point("worker", "start")
    assert start.x == pytest.approx(1600)
    assert start.y == pytest.approx(2600)
    assert config.direction("robot").y == pytest.approx(1)


@pytest.mark.parametrize("section,key,value", [
    ("robot", "speed", 0), ("worker", "delay_steps", -1),
    ("robot", "forward_duration", float("nan")), ("run", "max_steps", 0),
    ("zones", "stop_distance", 301), ("diagnostics", "crossing_epsilon", 0),
])
def test_invalid_config(section, key, value):
    data = ScenarioConfig.load().data
    data[section][key] = value
    with pytest.raises(ValueError):
        ScenarioConfig(data)


def test_config_file_load_and_snapshot(tmp_path):
    data = ScenarioConfig.load().data
    config = ScenarioConfig(data)
    data["robot"]["speed"] = 1
    assert config.data["robot"]["speed"] == 200
    file = tmp_path / "scene.json"
    file.write_text(json.dumps(config.data))
    assert ScenarioConfig.load(file).data == config.data


@pytest.mark.parametrize("x,event", [(600, True), (-1, False), (1601, False)])
def test_crossing_finite_segment(x, event):
    tracker = CrossingTracker(Point2D(0, 0), Point2D(1600, 0))
    tracker.update(Point2D(x, -100))
    result = tracker.update(Point2D(x, 100))
    assert result["worker_crossing_event"] is event
    assert result["worker_crossed_route"] is event


def test_touch_and_epsilon_band_do_not_count():
    tracker = CrossingTracker(Point2D(0, 0), Point2D(1600, 0))
    for y in (-100, -0.1, 0, 0.1, -50):
        assert not tracker.update(Point2D(600, y))["worker_crossed_route"]
    assert tracker.update(Point2D(600, 50))["worker_crossing_event"]
    assert tracker.update(Point2D(600, 100))["worker_crossed_route"]


def test_rotated_crossing():
    tracker = CrossingTracker(Point2D(10, 20), Point2D(10, 120))
    tracker.update(Point2D(-50, 70))
    assert tracker.update(Point2D(50, 70))["worker_crossing_event"]


def test_role_blueprint_override_and_fallback():
    config = ScenarioConfig.load()
    assert config.blueprint("worker") == "/Game/TrafficSystem/Pedestrian/Base_Pedestrian.Base_Pedestrian_C"
    assert config.blueprint("robot") == config.data["blueprint"]
    data = config.data
    del data["worker"]["blueprint"]
    assert ScenarioConfig(data).blueprint("worker") == data["blueprint"]


def test_unknown_role_blueprint_rejected():
    data = ScenarioConfig.load().data
    data["robot"]["blueprint"] = "/Game/Unknown.Unknown_C"
    with pytest.raises(ValueError, match="robot.blueprint"):
        ScenarioConfig(data)
