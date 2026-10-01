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

## Milestone 2: two-agent proximity scenario

`run_hrc_scenario.py` adds a separate scenario without changing the original
baseline commands or policies. **The robot-role agent is temporarily represented
by a humanoid visual placeholder**, not a construction robot model. The second
humanoid represents a worker. Both use the existing Base humanoid Blueprint.

- `agents/scripted_worker.py` waits for two scenario steps, executes twelve
  `forward 0.5` commands, then waits indefinitely. Constructor arguments allow
  a different delay, sequence, duration, and crossing length; `reset()` restarts
  the sequence. The worker never responds to the robot or distance zones.
- `state/hrc_state.py` defines typed, immutable `Point2D` and `HRCState` values.
- `state/state_extractor.py` reads each actor's live location and orientation
  using supported UnrealCV APIs, converts yaw to direction, and calculates
  Euclidean separation and inclusive proximity zones. The calculation function
  is simulator-independent and tested offline.
- `logging_utils.HRCLogger` writes unique `logs/hrc_<run_id>.json` files with
  `run_id`, run `metadata`, and a `states` array. Every entry contains all
  HRCState fields, including both actions. Files are flushed after each step.
  Original baseline logs retain their existing format.

The robot starts at `(0, 0)` facing +X and seeks `(1600, 0)`. The worker starts
at `(600, -600)` facing +Y, away from the robot's route. Both walking speeds are
set to 200; robot forward actions last 0.5 seconds. The nominal worker path
crosses the route at `(600, 0)`. The scenario uses this straight route to make
the crossing easy to inspect; the original baseline target remains unchanged.
IDs and names are distinct. Existing UE object names are checked and skipped
when allocating humanoids, avoiding reuse of actors from previous processes.
Role-to-ID/name mappings are printed and saved in metadata. Camera IDs are
allocated by the existing Humanoid class but no camera is read or used.

The caution threshold defaults to **300 Unreal units**, and the stop threshold
to **150 Unreal units**, with `distance <= threshold` counting as inside.
Both thresholds are **provisional implementation placeholders, not validated
construction-safety distances**. They will later be replaced by context-dependent
parameters. `stop=True` also implies `caution=True`. These are descriptive
state labels only: no slowing, stopping, safety rule, explanation, or dialogue
is implemented.

### Run on Windows

Start a fresh Base `demo_1` SimWorld backend first. Then:

```powershell
cd C:\Users\m09238yx\SimWorld-Construction
conda activate simworld
python hrc_project\run_hrc_scenario.py --max-steps 20
```

Optional scenario parameters:

```powershell
python hrc_project\run_hrc_scenario.py --max-steps 20 --worker-delay-steps 2 --caution-distance 300 --stop-distance 150
```

Expected console format (illustrative positions, not a guaranteed UE trace):

```text
robot_role (humanoid visual placeholder): GEN_BP_Humanoid_0, id=0
worker_role: GEN_BP_Humanoid_1, id=1
Step 6: robot=forward 0.5; worker=forward 0.5; robot_position=(600.0, 0.0); worker_position=(600.0, -200.0); distance=200.0; caution=True; stop=False
Caution-zone samples: ...
HRC log: C:\Users\m09238yx\SimWorld-Construction\hrc_project\logs\hrc_<run_id>.json
```

Expect the robot to advance, the worker to wait and cross, live separation to
change, and at least one `caution=True` sample if the installed scene supports
the intended path. The runner reports when no caution sample occurs rather
than claiming success. It continues to log after the robot reaches its target
so the whole worker sequence can be inspected. `task_phase` is
`worker_waiting`, `worker_crossing`, or `worker_finished` and tracks the scripted
command phase, not independently verified task completion.

Inspect the exact path printed by your run, or inspect the latest HRC log:

```powershell
$logFile = Get-ChildItem .\hrc_project\logs\hrc_*.json | Sort-Object LastWriteTime -Descending | Select-Object -First 1
$hrcRun = Get-Content -Raw -LiteralPath $logFile.FullName | ConvertFrom-Json
$hrcRun.metadata
$hrcRun.states | Select-Object simulation_step, robot_action, worker_action, human_robot_distance, worker_in_caution_zone, worker_in_stop_zone | Format-Table
```

### Limitations and manual verification

Verified on the installed Base backend on 2026-10-01: a 20-step live run spawned
distinct `GEN_BP_Humanoid_1` and `GEN_BP_Humanoid_2`. The worker crossed from
negative to positive Y at step 8. Caution was recorded at steps 6, 7, and 8;
step 7 separation was about 141.6 units and also triggered the stop-zone label.
Both roles continued their predetermined actions. The full state sequence was
written to `logs/hrc_77f69edebb084712b36f69facd068149.json` (ignored by Git).
All 38 offline tests, including unchanged baseline regressions, passed, as did
syntax compilation and CLI import/help checks. These observations verify this
installed backend, not physical safety or identical future trajectories.

Commands run sequentially: robot first, worker second, then both live poses are
sampled. This is one **scenario step**, not simultaneous motion or one UE
physics tick. Timestamp is UTC wall-clock sampling time. Pose reads are also
sequential. The fixed script is reproducible, but UE physics, frame timing,
terrain, collision geometry, and actual walking speed can change trajectories.
Endpoint sampling can miss proximity between samples. Both agents are
humanoids with collision enabled; their movement is not obstacle-aware.

All multi-agent methods are existing client APIs: `spawn_agent`,
`get_humanoid_name`, `get_objects`, `set_location`, `set_orientation`,
`humanoid_set_speed`, `humanoid_stop`, `humanoid_step_forward`,
`humanoid_rotate`, `get_location`, `get_orientation`, and `disconnect`.
Spawn presence is checked before movement. There is no camera-ID assumption,
new Blueprint, external API request, or LLM call.

For every fresh backend/version, verify two distinct names, the worker's +Y
heading, actual crossing of y=0, changing separation, and at least one caution
sample in the JSON. If a spawn or pose response fails, the runner stops and
preserves completed log entries. If the worker never crosses or caution never
occurs, inspect the scene and timing rather than interpreting the run as a
successful proximity scenario. Restart UE for comparable repeated runs;
disconnect does not remove actors, and old actors can obstruct later trials.

Run all regression and new tests with the unchanged command:

```powershell
python -m pytest hrc_project/tests -q
python -m compileall -q hrc_project
python hrc_project\run_hrc_scenario.py --help
```

## Next milestone

Obstacle/occlusion perception and structured context beyond proximity.
Obstacle assets, perception, dialogue, explanation generation, and safety
decision rules are not implemented in this milestone.
