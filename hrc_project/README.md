# First reproducible HRC baseline

This milestone provides two non-LLM policies for a single humanoid on the
existing SimWorld Base `demo_1` map. No API key, external API request, dialogue,
worker, construction object, perception model, or custom Unreal asset is used.
All implementation changes are isolated here; official examples remain intact.

## Architecture and scope

- `agents/scripted_agent.py`: configurable fixed commands. The default five-step
  sequence is `forward 1`, `forward 1`, `rotate 45 right`, `forward 1`, `wait`.
  It returns `None` once exhausted; `repeat=True` repeats indefinitely.
- `agents/deterministic_navigator.py`: measures Euclidean distance and heading
  error, turns toward the target with a capped turn, and moves forward only
  within heading tolerance. Defaults: goal radius 200 UE units, tolerance 5
  degrees, maximum turn 45 degrees, forward duration 1 second. Constructor
  arguments configure these values. This is direct goal seeking, **not
  obstacle-aware path planning**; it does not perform obstacle avoidance.
- `run_baseline.py`: owns a notebook-style `Environment` and CLI loop. It uses
  `Humanoid`, `Vector`, `Config`, `Map`, `Communicator`, and `UnrealCV` from the
  existing client. Spawn: `(0, 0, 600)`, direction `(1, 0)`, walking speed 200,
  target `(1700, -1700)`, Blueprint
  `/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C`.
  Roads are loaded from `data/example_city/demo_city_1/roads.json` using an
  absolute repository path. Neither policy needs the notebook's camera images,
  so the environment collects only position and direction.
- `logging_utils.py`: records each completed step's **post-action** pose, command,
  negative-distance reward, and action success. Each UTC timestamp is wall-clock
  sampling time. Each UUID run has a unique JSON array file in `logs/`; the file
  is flushed atomically after each step. Failed startup creates an empty array.
  A step interrupted before observation completes has no completed-step entry.
- `tests/`: offline pytest tests using simple x/y objects, without importing
  SimWorld or connecting to UE.

Each entry contains `run_id`, `timestamp`, `step`, `agent_type`, `position`
(`x`, `y`), `direction` (`x`, `y`), `action`, `reward`, `action_success`, and
`distance_to_target`. Generated JSON and temporary files are ignored by Git.
The runner prints the final log path even when startup or execution fails and
disconnects in `finally`. Disconnecting does not delete the spawned UE actor;
restart the backend before each comparable run to reset the world.

## Offline tests (Anaconda PowerShell Prompt on Windows)

```powershell
cd C:\Users\m09238yx\SimWorld-Construction
conda activate simworld
python -m pip install pytest
python -m pytest hrc_project/tests -q
python -m compileall -q hrc_project
python -m hrc_project.run_baseline --help
```

The existing SimWorld installation is used unchanged. This baseline adds no
OpenAI dependency. The existing `simworld/__init__.py` imports its LLM module
indirectly, but this baseline neither instantiates it nor calls it.

## Live runs

First start the already-installed `Windows\Windows\SimWorld.exe`, using the
Base map `/Game/Maps/demo_1`, and wait for the scene to load. Keep UE running.
If using the editor instead, enter Play mode. In Anaconda PowerShell Prompt:

```powershell
cd C:\Users\m09238yx\SimWorld-Construction
conda activate simworld
Test-NetConnection 127.0.0.1 -Port 9000
python -m hrc_project.run_baseline --agent scripted --max-steps 5
```

`TcpTestSucceeded` must be `True`. The first scripted run executes the default
five commands once; it is a movement/connection baseline, not a route guaranteed
to reach the target. To reproduce the notebook's repeating policy:

```powershell
python -m hrc_project.run_baseline --agent scripted --repeat --max-steps 100
```

Restart the backend to reset the world, then run the navigator:

```powershell
python -m hrc_project.run_baseline --agent deterministic --max-steps 100
```

Both modes stop at the goal radius or maximum step count. Scripted mode also
stops when its sequence ends. The navigator itself returns `wait` at the goal.

## API assumptions and manual verification

The inspected UnrealCV wrapper sends positive turn angles for `right` and
negative angles for `left`. Unreal yaw increases toward +Y, so positive heading
error selects `right`, contrary to the notebook's descriptive left/right text.
The actual Blueprint rotation must be checked on your installed backend.
Movement methods do not return UE acknowledgement: `action_success` indicates
a valid command returned without a Python exception, not verified goal progress
or collision-free motion. The client uses real-time sleeps; a fixed policy does
not guarantee identical trajectories under differing UE physics/frame timing.

When live verification is unavailable:

1. Start a fresh Base `demo_1` backend and confirm port 9000 is reachable.
2. Run the five-step scripted command above. Confirm spawn, forward movement,
   right turn, wait, five printed steps, and a JSON file with five entries.
3. Restart UE and run deterministic mode. From `(1, 0)` toward `(1700, -1700)`,
   expect an initial left turn and subsequent movement toward the target.
   Confirm heading error and distance reduce; obstacles may prevent reaching it.
4. Run again and confirm a different log filename and preserved earlier logs.
5. Press Ctrl+C during a longer run; confirm the log path is printed, earlier
   completed steps remain readable, and the Python connection closes.

## Next milestone

- Second humanoid worker.
- Obstacle/occlusion object.
- Structured HRC state extraction.

These extensions and dialogue-based safety explanations are not implemented.
