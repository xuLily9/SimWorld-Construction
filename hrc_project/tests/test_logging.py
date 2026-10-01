from datetime import datetime
import json
from types import SimpleNamespace as Point

from hrc_project.logging_utils import ExperimentLogger


def test_json_fields_and_unique_runs(tmp_path):
    logger = ExperimentLogger("scripted", tmp_path)
    obs = {"position": Point(x=3, y=4), "direction": Point(x=1, y=0)}
    target = Point(x=0, y=0)
    entry = logger.record(1, obs, target, "forward 1", -5, True)
    assert set(entry) == {"run_id", "timestamp", "step", "agent_type", "position", "direction",
                          "action", "reward", "action_success", "distance_to_target"}
    assert datetime.fromisoformat(entry["timestamp"]).utcoffset().total_seconds() == 0
    assert entry["position"] == {"x": 3.0, "y": 4.0}
    assert entry["direction"] == {"x": 1.0, "y": 0.0}
    assert entry["distance_to_target"] == 5
    assert entry["reward"] == -5
    assert entry["action_success"] is True
    assert json.loads(logger.path.read_text()) == [entry]
    logger.record(2, obs, target, "wait", -5, False)
    first_content = logger.path.read_text()
    assert len(json.loads(first_content)) == 2
    other = ExperimentLogger("scripted", tmp_path)
    assert other.path != logger.path
    assert json.loads(other.path.read_text()) == []
    assert logger.path.read_text() == first_content


def test_failed_startup_preserves_empty_log(tmp_path, monkeypatch, capsys):
    import pytest
    from hrc_project import run_baseline

    def unavailable(*args, **kwargs):
        raise OSError("offline")

    monkeypatch.setattr(run_baseline.socket, "create_connection", unavailable)
    monkeypatch.setattr(run_baseline, "ExperimentLogger",
                        lambda agent: ExperimentLogger(agent, tmp_path))
    with pytest.raises(RuntimeError, match="port 9000 is unavailable"):
        run_baseline.main(["--agent", "scripted", "--max-steps", "5"])
    files = list(tmp_path.glob("*.json"))
    assert len(files) == 1
    assert json.loads(files[0].read_text()) == []
    assert str(files[0].resolve()) in capsys.readouterr().out
