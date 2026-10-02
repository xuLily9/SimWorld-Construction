# HRC scenario

在自制施工地图 `/Game/HRConstruction/Model` 中运行两个角色：robot 按目标导航，worker 按固定脚本穿过 robot 路线。两者使用现有 humanoid Blueprint，robot 暂为视觉占位角色。程序读取实际 UE 位姿并记录距离、区域状态和穿越结果，不使用 LLM 或外部 API。

## 文件结构

| 文件 | 用途 |
| --- | --- |
| `scenario.json` | 场景几何、速度、延迟、导航和诊断参数 |
| `scenario.py` | 场景配置、坐标变换、执行循环和诊断 |
| `run_hrc_scenario.py` | CLI、配置覆盖和连接生命周期 |
| `environment.py` | 两种环境、角色创建、位姿观察和模型动作适配 |
| `agents/` | 目标导航、固定动作序列和 worker 策略 |
| `state/` | 状态结构、位姿提取和有限路线穿越检测 |
| `logging_utils.py` | 唯一运行编号及逐步写入的 JSON 日志 |
| `run_baseline.py` | 单角色连接及运动检查入口 |
| `tests/` | 不依赖 UE 的离线回归测试 |
| `logs/` | 历史实验数据，Git 忽略 |
| `reasoning/` | 独立推理资源，本次整理保持原样 |

## 运行 scenario

开发阶段先在安装了 UnrealCV 的 SimWorld Unreal 项目中打开 `Content/HRConstruction/Model.umap`，再进入 Play 模式。不要同时运行旧 SimWorld.exe，以免占用 9000 端口。每次可比较的实验应从新的 Play 会话开始，断开 Python 连接不会删除旧角色。打包程序只能打开已包含在该版本中的地图。

原始地图先准备能阻挡角色的平坦地面，覆盖 robot 和 worker 的整条路线，并添加照明方便观察。确认现有 `Base_User_Agent` Blueprint 及其依赖仍在项目中。当前配置保留 `spawn_z=600`，需要按实际地面高度调整；它不是自动贴地高度。Python 不会创建地面或照明，也不会自动加载目标地图。

在仓库根目录的 Anaconda PowerShell 中运行：

```powershell
conda activate simworld
Test-NetConnection 127.0.0.1 -Port 9000
python -m hrc_project.run_hrc_scenario --config hrc_project/scenario.json
```

`TcpTestSucceeded` 应为 `True`。也支持直接运行 `python hrc_project/run_hrc_scenario.py`。默认配置路径相对于模块目录解析。

CLI 参数可覆盖配置，最终生效值会保存到日志：

```powershell
python -m hrc_project.run_hrc_scenario --max-steps 20 --worker-delay-steps 2 --start-delay 5 --caution-distance 300 --stop-distance 150
python -m hrc_project.run_hrc_scenario --help
```

## 配置和执行

robot 使用目标导航策略和 `Base_User_Agent` 占位模型，worker 使用固定穿越策略和 `Base_Pedestrian` 行人模型。顶层 `blueprint` 提供默认模型，各角色的 `blueprint` 可覆盖默认值。动作解析由 `environment.parse_action` 共用，角色控制由 `environment.py` 按模型选择：humanoid 使用 `StepForward`，行人使用 `MoveForward` 并在指定时长后发送 `StopPedestrian`。角色模型与动作策略分别配置。

参数集中在 `scenario.json`。`map` 描述需要手动打开的地图，程序不会切换或验证当前 UE 地图；接受 `/Game/...` 地图路径，并自动去掉可选的 `.umap` 后缀。仍使用现有 `Base_User_Agent` Blueprint。scenario 不加载 demo 城市道路图，直接依靠观测朝向和坐标导航；单角色 baseline 仍保留原有 demo 道路配置。

- `origin.x/y` 平移整个布局，`origin.yaw_degrees` 同时旋转起点、目标和朝向。`origin.spawn_z` 是绝对 UE 高度，默认 600，换区域时需要检查地面与出生高度。
- 默认 robot 从 `(0, 0)` 前往 `(1600, 0)`；worker 从 `(600, -600)` 朝 +Y 前进。Unreal yaw 为 0 时朝 +X，为 90 时朝 +Y。
- worker 等待 `delay_steps` 步后，执行 `crossing_steps` 次固定时长前进，再持续等待。worker target 用于参考和奖励，不控制脚本终点。
- `run.start_delay_seconds` 是运行前检查角色的等待秒数。worker 延迟按 scenario 步计数，每步依次执行 robot、worker，然后采样位姿。
- `zones` 默认 caution 为 300、stop 为 150 UE 单位，边界包含在区域内。这些是观察标签及临时参数，不会触发减速或停止。
- `diagnostics` 控制静止运动告警和穿越检测容差。

导航器直接朝目标前进，不避障。命令成功只表示派发没有 Python 异常，实际运动以观测位姿为准。动作和位姿读取均顺序执行，时间戳为 UTC 墙钟时间；UE 时序、地形及碰撞会影响轨迹。

## 日志和结果检查

每次运行生成 `logs/hrc_<run_id>.json`，包含 `run_id`、`metadata` 和 `states`。每步保存实际位姿、动作、距离、脚本阶段、区域标签、位移和穿越诊断。metadata 保存生效配置、世界坐标、角色名称、初始位姿、已有 humanoid 及最终 `validation_summary`。

穿越检测要求 worker 观测位置出现在 robot 有限路线的两侧，且插值交点位于路线段内。触线、穿过路线延长线或完成脚本都不算穿越。`task_phase` 仅表示脚本阶段，采样与插值无法证明完整连续轨迹。

`scenario_validated` 要求两个角色都有实际运动、worker 穿越路线、至少出现一次 caution 采样，且 robot 到达目标容差范围。控制台会提示静止、缺少穿越、缺少 caution 或未到目标。

```powershell
$logFile = Get-ChildItem ./hrc_project/logs/hrc_*.json | Sort-Object LastWriteTime -Descending | Select-Object -First 1
$trial = Get-Content -Raw -LiteralPath $logFile.FullName | ConvertFrom-Json
$trial.metadata.validation_summary
$trial.states | Select-Object simulation_step, robot_displacement, worker_displacement, human_robot_distance, worker_crossing_event, worker_in_caution_zone, worker_in_stop_zone | Format-Table
```

现场确认两个新角色的名称、朝向、出生高度和路线畅通。无运动时检查 UE 暂停状态、控制器与碰撞；无穿越时检查 worker 朝向和前进时长；无 caution 时检查几何及延迟。增加最大步数不会延长 worker 的固定前进序列。

## 单角色 baseline

保留此入口用于检查连接、运动和导航。默认出生点 `(0, 0, 600)`、目标 `(1700, -1700)`、速度 200。

```powershell
python -m hrc_project.run_baseline --agent scripted --max-steps 5
python -m hrc_project.run_baseline --agent scripted --repeat --max-steps 100
python -m hrc_project.run_baseline --agent deterministic --max-steps 100
```

scripted 默认依次执行 `forward 1`、`forward 1`、`rotate 45 right`、`forward 1`、`wait`。序列结束、到达目标或达到步数上限时退出。日志为 `logs/baseline_<run_id>.json`，使用 JSON 数组保存完成步骤的动作、位姿、奖励和派发结果。

## 离线验证

```powershell
python -m pytest hrc_project/tests -q
python -m hrc_project.run_baseline --help
python -m hrc_project.run_hrc_scenario --help
```

测试覆盖动作策略、配置变换、区域边界、穿越检测、日志及模拟执行顺序。实际 UE 出生、运动和碰撞仍需现场验证。
