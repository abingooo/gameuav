# SPF 与 SMPF 六类任务实现说明

本文基于 2026-07-19 的本地代码，说明 SPF（See-Point-Fly）和 SMPF
（See-Model-Plan-Fly）如何处理导航、避障、长时域、推理、搜索和跟随六类任务。
本文只描述代码实际存在的机制，不把单元测试、干跑规划或一次链路动作写成已完成的
真机能力。

> 术语说明：用户常说的“长视野”在论文中是 Long-Horizon，本文统一写作
> “长时域/长程任务”。它表示一条指令包含多个按顺序执行的阶段，不表示相机看得更远。

## 1. 先给结论

1. SPF 没有六套任务算法。六类任务都是不同自然语言提示词驱动同一个闭环：
   看当前 RGB、让 VLM 选择一个二维点并估计距离、转换为一次相对位移、到点后重新看图。
2. SMPF 虽然接受六个 `mode`，也不是六套完全独立的算法。
   `navigate`、`obstacle`、`reasoning` 共用同一套语义度量规划管线；
   `long_horizon` 在其上增加阶段和目标身份状态；`search` 是有限原地偏航扫描；
   `follow` 直接计算一个三维驻点，不让 LLM 生成路径。
3. SPF 与 SMPF 的最大结构差异不是“多调用一次大模型”，而是控制权分层不同：
   SPF 的 VLM 点直接成为 PX4Ctrl 的局部位置目标；SMPF 先建立 RGB-D 三维模型，
   验证语义路线，再把目标交给 EGO 生成实时避障轨迹。
4. 当前正式真机对比只有五类。SPF 论文的 Search 仅在仿真中评测，因此本地真机清单
   排除 Search。正式计划是 11 条提示词 × 5 次重复 × 2 种方法 = 110 次飞行。
5. 当前还不能说 SPF 或 SMPF 已经“完成六类真机任务”。正式 outcome 文件尚不存在；
   现有代码、测试和 SMPF dry-run 只能证明链路或算法部件可运行。

## 2. 总览对比

| 任务 | SPF 的实现 | SMPF 的实现 | 核心差别 |
| --- | --- | --- | --- |
| 导航 Navigation | 当前 RGB 上选目标点，VLM 估计 1-10 级深度，执行一个相对动作后重复原指令 | RGB-D 建目标/障碍三维球，LLM 给 guidepoints，确定性验证或 A* 修复，再交给 EGO | 反应式单点控制 vs. 有度量约束的分层规划 |
| 避障 Obstacle Avoidance | 主基线 `adaptive_mode` 没有障碍几何；另有 `obstacle_mode`，但障碍框只记录和画图 | 语义障碍转三维球，筛选接近进场走廊的障碍，验证连续线段；EGO 使用完整实时深度图规划轨迹 | 仅依赖 VLM 对原始指令的隐式反应 vs. 语义几何验证 + EGO 实时避障 |
| 长时域 Long-Horizon | 每轮重复整条复合指令，无显式阶段和历史目标身份 | 分解为 2-5 个有序阶段，到达后才推进；记录已完成目标 ID，后续观测若关联为该 ID 则拒绝 | 隐式依赖 VLM 重解释 vs. 显式阶段状态机 |
| 推理 Reasoning | VLM 在当前 RGB 中直接选择最符合自然语言需求的对象 | VLM 负责从可见对象中推断目标；后续 LLM 只规划度量路线 | 两者的语义推理都主要由 VLM 完成，SMPF 多了可验证执行层 |
| 搜索 Search | 作者仅在仿真中把搜索提示词交给同一单点闭环；无 not-found 分支和搜索状态机 | 未看到目标时保持位置，依次扫描 7 个偏航视角；检测到即成功 | 无显式搜索策略 vs. 有限原地扫描，但都不是完整空间搜索 |
| 跟随 Follow | 到达一个动作点后重新检测目标，再生成下一个动作；无跟踪器和运动预测 | 用新鲜 RGB-D/SAM 估计目标球，选择球面外一个三维驻点，交给 EGO；到点后重新观测 | 低频 VLM 投点 vs. 低频三维重定位与驻点跟随 |

## 3. 当前版本、模型和输入

### 3.1 SPF

- 作者代码固定在 commit `5621bcf43e9826d60df014541dd0498e743a92bd`。
- 本地主基线模式是 `adaptive_mode`，不是 `obstacle_mode`。
- 本地视觉模型是 `gemini-3.5-flash`。
- 图像输入是 `/rgb1/image_raw`，没有使用 RealSense 深度作为 SPF 策略输入。
- VINS odometry 用于把相对动作转换到世界坐标、检查到达状态；它不进入作者 VLM 提示词，
  也不构成任务记忆。

配置依据：`strategy/see_point_fly/adapter/config_tello.yaml:6-9`。

### 3.2 SMPF

- SMPF 来自 `muavold` 的 `dev` 分支原型，导入基线 commit 为
  `9c0121ae60722d5e1db7d99380cb2cd734aab48a`；当前 GameUAV 代码已经补齐并约束了
  原型中不可靠的坐标、同步、验证和执行部分。
- 视觉目标落地模型是 `gemini-3.5-flash`。
- guidepoint 规划和长时域阶段分解模型是 `gpt-5.2`，`reasoning_effort=low`。
- SAM 服务用于把目标/障碍从框细化为掩码；当前部署地址是 `10.246.1.94:5002`。
- 输入是同步的 RealSense 彩色图、对齐深度和彩色 CameraInfo，并结合 VINS odometry 与
  相机外参。
- `follow` 不调用 `gpt-5.2` 规划路径；`search` 找到目标即结束，也不进入三维路线规划。

默认模型依据：`strategy/smpf/runtime/model_defaults.py:5-8`。输入与执行参数依据：
`ros_nodes/mission/smpf_bridge/launch/smpf_bridge.launch:2-14`。

## 4. SPF 的共同任务机制

### 4.1 感知和动作生成

SPF 每一轮只求一个 `ActionPoint`：

```text
/rgb1/image_raw + 未改写的自然语言指令
    -> gemini-3.5-flash
    -> 一个归一化二维点 [y, x] + VLM 深度等级 1..10
    -> TelloActionProjector 反投影
    -> ActionPoint(right, forward, up)
```

`adaptive_mode` 提示模型寻找与整条指令最匹配的对象，通常选择最大或最近的匹配对象，
把点放在对象中心，并根据对象在画面中所占比例估计深度等级。这里的深度不是深度相机
测量值。`depth <= 2` 时动作被标为只调整朝向；更远的深度经过非线性缩放后生成位移。

对应实现：

- `strategy/see_point_fly/upstream/src/spf/tello/action_projector.py:77-101`
- `strategy/see_point_fly/upstream/src/spf/tello/action_projector.py:242-263`
- `strategy/see_point_fly/upstream/src/spf/tello/action_projector.py:361-390`
- `strategy/see_point_fly/worker/spf_worker.py:465`

### 4.2 真机控制链

```text
ActionPoint(right, forward, up)
    -> 使用当前 VINS 朝向转为 world/ENU 目标并限幅
    -> /control/spf_position
    -> gameuav_control_interface
    -> /control/position_cmd
    -> px4ctrl
```

当前每步水平位移最多 `1.5 m`、垂直位移最多 `0.3 m`，目标高度限制在
`0.4-1.5 m`。`goal_projection_enabled=false`，所以主 SPF 链路明确绕过 EGO，
也不使用 EGO occupancy cloud 修正目标点。

对应实现：

- `launch/bringup_see_point_fly.launch:2-15`
- `ros_nodes/mission/see_point_fly_bridge/scripts/see_point_fly_bridge.py:384`
- `ros_nodes/mission/see_point_fly_bridge/launch/see_point_fly_bridge.launch:26-47`
- `ros_nodes/control/gameuav_control_interface/scripts/control_interface_node.py:195-252`

### 4.3 SPF 循环和完成语义

```text
IDLE
  -> WAITING_GOAL：发布同一条原始指令，等待一个 SPF 位置目标
  -> WAITING_ARRIVAL：等待局部目标的位置、速度和稳定时间条件
  -> WAITING_NEXT：延时后用同一条指令开始下一轮
  -> WAITING_GOAL
```

局部目标到达只意味着“可以再看一张图并生成下一步”，不意味着任务完成。SPF 代码没有
自动判断“已经找到正确的人”“已经完成两阶段任务”或“跟随距离正确”；必须由操作员发送
`complete`/`success` 才进入任务级 `SUCCESS`。循环还会因单点超时、总任务超时、里程计
失效或达到最大循环数而结束。

对应实现：`ros_nodes/mission/see_point_fly_bridge/scripts/spf_task_executor.py:165-241,269-282`。

## 5. SMPF 的共同任务机制

### 5.1 静态任务共用管线

除 Search 和 Follow 的特殊分支外，SMPF 使用下面的共同管线：

```text
同步 RealSense RGB-D + CameraInfo + VINS/外参
    -> VLM 输出一个目标框和最多 8 个可见障碍框
    -> SAM 掩码 + 对齐深度的稳健统计
    -> 带不确定性膨胀的三维物体球
    -> body FLU / world ENU 变换和世界系语义记忆
    -> 只选接近“当前位置到目标驻点”走廊的语义障碍
    -> gpt-5.2 输出 body-FLU guidepoints
    -> JSON schema、点、连续线段、边界、终点距离、进度和目标可见性验证
    -> 两次 LLM 路线都失败时，visibility graph + A* 确定性修复
    -> 世界系 waypoint + 朝向目标的 yaw
    -> /control/ego_position -> EGO -> px4ctrl
```

这里大模型只负责两件事：VLM 决定“图中哪个对象符合指令”，规划 LLM 提议“路线形状”。
它们都不是数值安全的最终裁决者。代码会独立检查输出结构、每个点和每条连续线段；
如果模型路线不合法则拒绝或修复。EGO 再依据完整实时深度图生成和更新实际飞行轨迹。

### 5.2 坐标链

```text
彩色像素 + 对齐深度 + 彩色内参
    -> color optical (right, down, forward)
    -> RealSense depth/infra1 外参链
    -> VINS 相机到机体外参
    -> body FLU (forward, left, up)
    -> VINS 姿态和位置
    -> world ENU
```

SMPF 使用在线 CameraInfo 和外参，而不是导入原型中的硬编码相机参数。RGB-D 同步帧过旧、
图像尺寸不匹配、外参缺失或平移量明显不合理时，规划会拒绝继续，不会猜测坐标。Follow
还显式要求 RGB-D/odom 时间偏差不超过 `0.08 s`；普通静态模式目前没有同等的显式
RGB-D/odom skew 检查，不能把 Follow 的时间约束泛化成所有模式都已具备。

### 5.3 三维对象和路线验证职责

- 目标必须经过 SAM 和对齐深度建模，目标分割失败会终止本轮。
- 可选障碍的 SAM 失败时，可以保守地使用完整 VLM 框和对齐深度建立球体；不会悄悄删除
  一个已经检测到的障碍。
- 三维球存入世界系语义记忆，并按归一化标签和中心距离关联到 `object_id`。静态目标的默认
  关联门限是 `0.35 m`，Follow 动态目标使用 `1.50 m`；标签漂移或位置变化超过门限会产生
  新 ID，因此这不是可靠的跨视角 re-identification。
- 语义规划只纳入距直达进场走廊表面余量不超过 `0.25 m` 的障碍，避免让远离路线的
  物体干扰 LLM/A*。这不影响 EGO 使用完整深度地图避障。
- 普通目标终点必须位于膨胀目标球表面外 `0.15-1.00 m`，产生至少 `0.10 m` 的目标
  进度，并保留到目标的可见线；否则路线不被接受。

主要实现：

- `strategy/smpf/runtime/vision_detector.py:123-184`
- `ros_nodes/mission/smpf_bridge/scripts/smpf_bridge.py:1024-1203`
- `strategy/smpf/runtime/scene_memory.py:37`
- `strategy/smpf/runtime/obstacle_relevance.py:27`
- `strategy/smpf/runtime/model_planner.py:208-401`
- `strategy/smpf/runtime/goal_validation.py:29`

### 5.4 控制链和执行门

```text
SMPF world goal
    -> /control/ego_position
    -> control_interface 发布 /planning/goal
    -> EGO planner / traj_server
    -> /control/ego_position_cmd
    -> control_interface
    -> /control/position_cmd
    -> px4ctrl
```

发布目标前必须同时满足：launch-time `execution_enabled=true`、运行时执行开关已打开、
PX4 已连接并解锁、VINS 新鲜且起始高度符合限制；打开运行时执行门时还会检查速度。
默认 launch 参数仍是
`execution_enabled=false`。因此 dry-run 可以发布已验证规划到 `/smpf/dry_run_plan`，但不会
发布控制目标。

执行器对每个 waypoint 检查水平误差、垂直误差、速度、目标朝向 yaw 和稳定时间；
这些条件只证明 waypoint 执行完成，不自动等于论文语义任务成功。

另有一个操作员覆盖入口：向 `/smpf/task_control` 发送 `complete` 或 `success` 会直接清空
任务状态并发布 `SUCCESS`，不检查下面各模式的自动完成条件。当前实现也没有在该分支中
abort 正在运行的 executor，因此它不能被理解为安全停止命令，更不能直接作为正式实验
outcome。本文后续所说的 SMPF“自动成功”均不包含这个覆盖入口。

主要实现：

- `ros_nodes/mission/smpf_bridge/scripts/smpf_bridge.py:626-646`
- `ros_nodes/mission/smpf_bridge/scripts/smpf_bridge.py:1696-1720,2056-2075`
- `strategy/smpf/runtime/execution.py:69-166`
- `ros_nodes/control/gameuav_control_interface/scripts/control_interface_node.py:277-301`

## 6. 六类任务逐项说明

### 6.1 导航 Navigation

任务目标示例：`Fly to the chair (long distance)`。

#### SPF

1. 把完整导航指令和当前 RGB 交给 `adaptive_mode`。
2. VLM 选择椅子的中心点，并用 1-10 等级估计距离。
3. 投影器生成一次 `right/forward/up` 相对位移。
4. 桥接节点把位移转成一个世界系局部位置目标，直接交给 PX4Ctrl。
5. 到点稳定后重新拍图，并再次使用完全相同的指令。
6. 操作员确认语义目标已经完成后，发送 `complete`。

SPF 没有语义目标的持久世界坐标、全局地图或全局路线。所谓长距离导航来自“多次局部
看图和移动”，不是一次规划完整长距离路径。

#### SMPF

1. `navigate` 使用场景 VLM 从当前彩色图中找出目标，同时可落地进场走廊附近的障碍。
2. SAM、对齐深度和外参把目标变为世界系三维球。
3. 根据目标球计算可接受的终点距离带，并让 `gpt-5.2` 提议局部 guidepoints。
4. 确定性代码检查路径连续段、边界、碰撞余量、终点进度和目标可见性。
5. 若两次模型路线均不合法，使用三维 visibility graph + A* 尝试修复。
6. 通过验证的 waypoint 依次交给 EGO；EGO 负责生成实际轨迹和对实时深度障碍作反应。

`navigate` 的自动 `SUCCESS` 是所有 waypoint 达到位置/速度/yaw/稳定条件。正式实验仍需
由操作员按无碰撞、目标可见和距离规则评分。

### 6.2 避障 Obstacle Avoidance

任务目标示例：`Fly to the person without hitting the cone`。

#### SPF

主对比基线仍使用 `adaptive_mode`。因此“without hitting the cone”只是原始提示词的一部分，
VLM 可能通过语义理解选择一个看起来较安全的目标点，但代码没有建立圆锥的三维几何，
也没有检查从当前位置到该点的连续路径。

作者另有 `obstacle_mode`：

1. 提示 VLM 返回目标点和障碍框，并要求在必要时“slightly adjust”目标点。
2. 该模式用固定 `1.1` 深度反投影目标点，而不是返回度量障碍距离。
3. 障碍框附加到 `ActionPoint`，用于日志和可视化。
4. 控制器仍然只执行一个目标点；框没有进入几何碰撞检测或轨迹规划。

所以 `obstacle_mode` 不能被描述为 EGO 等价的避障器，并且如果测试它，必须作为独立
variant 报告，不能混入主基线 `spf_adaptive`。

对应实现：`strategy/see_point_fly/upstream/src/spf/tello/action_projector.py:211-239,288-334`。

#### SMPF

`obstacle` 与 `navigate` 共用同一规划实现，模式名本身不会切换到另一套规划器。差异主要
来自指令明确命名了不可碰撞对象：

1. 场景 VLM 分开输出目的目标和可见的具体障碍。
2. SAM/RGB-D 把障碍变成带安全膨胀的三维球。
3. 走廊相关性模块只把可能影响当前进场路线的语义球交给 LLM/A*。
4. 验证器检查所有 guidepoint 和相邻点之间的连续线段，而不是只检查终点。
5. EGO 使用完整的实时深度 occupancy 负责实际轨迹级避障，包括没有被 VLM 命名的障碍。

分工应准确表述为：SMPF 决定语义目标、语义障碍和满足当前模型检查的度量目标/路线约束；
EGO 才是实际轨迹生成器。几何单测通过不等于真机无碰撞能力已经验证。

### 6.3 长时域/长程 Long-Horizon

任务目标示例：`Fly to the chairs and the next`。

#### SPF

SPF 不分解指令。每到达一个局部动作点后，仍把整句 `Fly to the chairs and the next`
交给当前新图像。VLM 必须仅依靠此刻视野自行解释现在该选“第一个”还是“下一个”。

代码中没有：

- 阶段数组和阶段索引；
- 第一阶段完成检测；
- 已访问目标 ID；
- 历史图像或显式语义记忆；
- 防止再次选择同一把椅子的规则。

因此 SPF 的长时域能力是通用 VLM 闭环在复合提示词上的零样本表现，最终完成由操作员
判断，而不是一个专门长时域规划器的结果。

#### SMPF

1. `gpt-5.2` 把原始指令严格分解为 2-5 个有序、每阶段只含一个可视觉落地目标的阶段。
2. 阶段分解完成后必须获取时间戳更新的 RGB-D 帧，避免用模型调用前的旧图做第一阶段落地。
3. 当前阶段按普通静态管线完成 VLM/SAM/RGB-D 建模、guidepoint 规划、验证和 EGO 执行。
4. 只有 waypoint 的位置、速度、yaw 和稳定条件全部满足后，阶段索引才前进。
5. 完成目标的 `object_id` 被写入状态。后续观测若按标签和中心距离再次关联到这个已完成
   ID，代码拒绝该目标，并启动有限偏航扫描寻找不同对象；若标签或位置漂移导致新 ID，
   该保护不能识别它仍是旧对象。
6. 所有阶段均执行完成后，自动状态机才进入 `SUCCESS`。

阶段契约只允许 `completion="reach_target"`。它解决的是顺序和目标身份问题，并不等同于
高层形式化任务规划，也不会自动证明论文语义成功。完整多阶段真机飞行尚未验证。

对应实现：

- `strategy/smpf/runtime/task_stages.py:29-101`
- `ros_nodes/mission/smpf_bridge/scripts/smpf_bridge.py:811-857,1099-1143,1999-2020`

### 6.4 推理 Reasoning

任务目标示例：`I'm thirsty, find something that can help me.`。

#### SPF

模糊需求原样进入 `adaptive_mode`。VLM 直接在当前图像中选择它认为能满足需求的对象，
然后仍按普通二维投点和深度等级生成动作。SPF 没有独立知识库、符号推理模块、任务分类器
或可供验证的推理步骤。

#### SMPF

1. `reasoning` 仍使用和 `navigate` 相同的场景检测入口。
2. 场景提示明确要求 VLM 推断当前画面中能满足需求的可见物体，并把它作为唯一目标。
3. 若目标不在画面中，VLM 必须返回空目标；`reasoning` 随后停止本轮并报错，不猜测
   离屏位置，也不会自动切换到 Search。
4. 目标落地以后，`gpt-5.2` 只根据三维球和边界提议如何飞过去，不重新决定哪个物体
   满足“解渴”“需要帮助”等语义。
5. 路线按静态管线验证并交给 EGO。

所以两种方法的“推理”本质上都首先是 VLM 的视觉语义选择。SMPF 的增量是把选择结果
转成可测量、可拒绝和可交给 EGO 的执行目标，而不是增加一个可解释的符号推理器。

对应实现：`strategy/smpf/runtime/vision_detector.py:165-184` 和
`ros_nodes/mission/smpf_bridge/scripts/smpf_bridge.py:1722-1724`。

### 6.5 搜索 Search

这里必须区分“作者论文类别”和“本地真机范围”。SPF 的 Search 只在 DRL Simulator 中
报告，本地没有把 Search 加入真机对比。

#### SPF

作者仿真代码仍把搜索指令交给同一个 VLM 单点投影入口。提示格式要求模型在当前图像中
选点，没有明确的 `not_found` 返回、航向扫描序列、覆盖率地图、已搜索区域记忆或 frontier
exploration。因此它不能被解释为一个独立的搜索状态机。

对应实现：

- `strategy/see_point_fly/upstream/src/spf/sim/action_projector.py:121-205`
- `strategy/see_point_fly/upstream/src/spf/sim/main.py:209-234`

#### SMPF

`search` 是一个明确但范围有限的状态机：

1. 在当前图像中运行目标检测；若检测到目标，立即返回 `SUCCESS`。
2. 若未检测到，并且任务提交时请求了执行、所有执行门也保持打开，则保持当前
   `x/y/z` 不变，只改变 yaw。
3. 相对初始 yaw 依次尝试 `+45, -45, +90, -90, +135, -135, 180` 度。
4. 每个视角等待 yaw 误差不超过 `10 deg`、速度不超过 `0.25 m/s` 并稳定 `0.5 s`，
   或等待该视角超时，然后重新取图检测。
5. 七个视角仍未检测到目标则 `TIMEOUT`。

该模式的自动成功定义只是“看见目标”，不是“飞到目标”。它不平移、不做房间覆盖、frontier
搜索、三维搜索路径或跨位置的搜索记忆。因此准确名称是“有限原地七视角扫描”，不能写成
完整自主搜索。它保留在代码中，但不进入当前真机 SPF/SMPF 对比。

对应实现：`ros_nodes/mission/smpf_bridge/scripts/smpf_bridge.py:906-920,1722-1755,1822-1850`。

### 6.6 跟随 Follow

任务目标示例：`Fly toward the person with green shirt`。

#### SPF

SPF 没有专门跟随控制器。一次局部动作到达后，它重新获取图像并再次用原目标描述做 VLM
投点。移动目标在新图中的位置变化会产生新的相对动作，因此表现为低频视觉闭环。

它没有：

- 连续图像目标 tracker；
- 跨帧身份记忆或遮挡恢复；
- 目标速度估计；
- 运动预测；
- 明确的三维跟随距离控制。

#### SMPF

Follow 特意不让规划 LLM 生成 guidepoint 路线：

1. VLM 先在 grounding frame 中识别目标标签。
2. VLM 返回后，系统原子获取更新的 RGB-D、VINS 和有效外参快照。度量帧年龄必须
   `<= 1.0 s`，RGB-D 与 odom 偏差必须 `<= 0.08 s`。
3. 用目标标签对更新后的整幅彩色图调用 SAM。至少需要一个掩码；有多个掩码时按像素面积
   取最大，面积相同时取返回顺序中的第一个。SAM 调用预算最多 `0.75 s`，并受剩余新鲜度
   预算约束。
4. 由所选掩码、对齐深度和外参建立当前目标三维球。
5. 在目标安全球表面外选择一个自由三维驻点。默认要求表面间距 `0.15 m`；若安全余量
   更大，则安全余量优先。优先选择观察者一侧的点，若该点被占用或越界则尝试其他球面点。
6. 只检查驻点本身是否在边界内、是否占用以及能否保持目标可见。代码不把无人机到驻点
   的直线当作实际轨迹。
7. 将唯一世界系驻点发布给 EGO，由 EGO 生成完整轨迹并处理实时障碍。
8. 到达后等待 `0.5 s` 再获取新观察。目标球表面距离处于 `0.15 +/- 0.10 m` 才成功；
   尚未满足则生成新的驻点。循环次数用尽后还会做最终观察，不满足则 `TIMEOUT`，不会
   把“跑满次数”记成成功。

主方法中 `follow_step_limit_enabled=false`，因此不会把每轮目标截断到 `0.50 m`；
`0.50 m` 上限只保留为 `smpf_bounded_follow_goal` 消融。SMPF 仍是离散“观察-飞到驻点-
重观察”，不是连续高频 tracker，也没有目标运动预测。当前目标一开始不可见时，直接跟随
分支会报错，不应声称它具备遮挡搜索恢复。

此外，新帧 SAM 只接收 VLM 给出的目标标签，不使用上一帧目标框做跨帧身份绑定。如果
画面中有多个同标签对象，面积最大掩码规则可能切换到另一个对象；当前代码没有 tracker
来消除这种身份歧义。

对应实现：

- `ros_nodes/mission/smpf_bridge/scripts/smpf_bridge.py:926-1023,1211-1296,1984-1998`
- `strategy/smpf/runtime/follow_policy.py:39-199`
- `ros_nodes/mission/smpf_bridge/launch/smpf_bridge.launch:51-57`

## 7. 两种方法的任务完成含义

| 层次 | SPF | SMPF |
| --- | --- | --- |
| 单个局部目标完成 | 到达一个直接位置点并稳定，然后启动下一次 VLM 推理 | 到达一个 EGO waypoint，并满足位置、速度、yaw、稳定时间 |
| 普通任务自动成功 | 不自动判断 | Navigate/Obstacle/Reasoning 的全部已验证 waypoint 执行完成 |
| 长时域自动成功 | 不自动判断 | 所有有序阶段的 waypoint 执行完成 |
| Search 自动成功 | 作者代码无独立本地真机完成逻辑 | 当前 RGB 检测到目标；不要求接近目标 |
| Follow 自动成功 | 不自动判断 | 新鲜重观察中，目标可见且球表面距离进入驻点容差带 |
| 操作员 `complete`/`success` | 正常任务完成入口，将 TaskLoop 置为 `SUCCESS` | 强制状态覆盖；不检查模式条件，且当前代码不 abort 活跃 executor |
| 正式实验成功 | 无碰撞，并满足论文任务与最终视图/距离规则 | 使用完全相同的操作员规则，不以内部状态替代人工评分 |

正式真机规则是：无碰撞地完成请求；适用时最终目标清晰可见且距离不超过 `1 m`。
因此以下内容都不能单独记为正式成功：

- VLM 返回了一个点或目标框；
- SAM 得到了掩码；
- LLM 路线通过几何检查；
- EGO 接受了一个目标；
- 无人机到达一个局部 waypoint；
- dry-run 输出了规划 JSON；
- 单元测试通过。

## 8. 当前能力证据边界

截至本文核查时间：

| 项目 | 已有证据 | 仍缺少的证据 |
| --- | --- | --- |
| SPF 作者源码 | 作者仓库内容和 commit 完整性已核对；本地 worker 使用作者投点器 | 五类固定真机任务的正式重复结果 |
| SPF 真机链路 | RGB1 -> SPF -> `/control/spf_position` -> PX4Ctrl 链路已核查；有过非正式动作 | 按清单完成 Navigation/Obstacle/Long-Horizon/Reasoning/Follow 的 operator outcome |
| SMPF 算法组件 | 坐标、同步、分割、深度球、验证、A*、阶段身份、Follow 驻点有单测和 dry-run | 多阶段推进、移动目标跟随和 EGO 执行的完整真机验证 |
| Search | 作者仿真提示词清单保留；SMPF 有七视角扫描代码 | 不属于本地真机比较，不能记成已复现的真机能力 |
| 正式对比 | 固定清单、profile、记录和汇总工具已存在 | `runtime/spf_smpf_outcomes.jsonl` 尚不存在，正式进度为 0/110 |

最近一次 SPF 飞行中的下降已由操作员确认是电池电量耗尽，不应被误记为 SPF 坐标链故障；
但它同样不能作为任何任务成功证据。

## 9. 如何公平解释 SPF 与 SMPF 对比

### 9.1 正式范围

主实验只包含：Navigation、Obstacle Avoidance、Long-Horizon、Reasoning、Follow。
清单中共有 11 条作者真机提示词，每条 5 次，两种方法成对测试，共 55 对、110 次。
Search 单独保留为作者仿真来源，不加入真机总表。

任务清单：`strategy/smpf/experiments/spf_realworld_tasks.json:13-24`。
实验 profile：`strategy/smpf/experiments/comparison_profile.json:4-23`。

### 9.2 能控制的变量

- 使用清单中的原始提示词，不改写；
- 相同场景布置、起始位姿和任务超时；
- 两种方法使用相同的操作员成功判据；
- SPF/SMPF 执行顺序随机化；
- 每轮记录碰撞、完成时间、实际路径、API 次数、最终目标可见性和最终距离；
- 每轮只允许选中的方法持有控制链。

### 9.3 必须报告的硬限制

1. **相机不同。** SPF 保留 RGB1，SMPF 使用 RealSense 彩色图和深度。这是已明确保留的
   方法差异，因此结果不是 camera-controlled comparison，不能只归因于规划方法。
2. **下游控制不同。** SPF 直接向 PX4Ctrl 提交局部位置目标；SMPF 把目标交给 EGO。
   因此主实验比较的是两个端到端系统，而不是纯粹隔离的“大模型规划器”。
3. **模型调用结构不同。** 两者视觉决策都用 `gemini-3.5-flash`，但 SMPF 静态任务还使用
   SAM 和 `gpt-5.2`。Follow 不调用规划 LLM。必须分别记录调用次数和延迟。
4. **平台不是原论文数值复现。** 作者 SPF 在 DJI Tello EDU 上使用定时 RC 动作；本地把
   策略输出接到了 VINS/PX4 位置控制。应称为作者策略的本地平台移植。
5. **`obstacle_mode` 不是主 SPF。** 主表固定为 `spf_adaptive`；若运行 `obstacle_mode`，
   只能作为另一个 variant 或消融单独报告。

## 10. 六类任务的模型调用归属

| 模式 | 视觉 VLM | SAM/RGB-D | 规划 LLM | EGO | 显式任务状态 |
| --- | --- | --- | --- | --- | --- |
| SPF 六类 | 每轮生成 1 个动作；调用或解析失败时本轮无动作 | 否 | 否 | 否 | 只有通用局部动作循环 |
| SMPF Navigate | 目标 + 障碍落地 | 是 | guidepoints | 是 | 单阶段执行 |
| SMPF Obstacle | 目标 + 障碍落地 | 是 | guidepoints | 是 | 与 Navigate 共用 |
| SMPF Long-Horizon | 每阶段落地 | 是 | 阶段分解 + 每阶段 guidepoints | 是 | 阶段索引 + 已完成目标 ID |
| SMPF Reasoning | VLM 推断满足需求的可见目标 | 是 | 只规划路线 | 是 | 与 Navigate 共用 |
| SMPF Search | 每个搜索视角检测目标 | 未找到时不建模；找到即结束 | 否 | 仅用于原地偏航目标 | 七视角索引 |
| SMPF Follow | grounding label | 新帧全图 SAM + 深度球 | 否 | 唯一三维驻点的轨迹 | cycle + 最终重观察 |

## 11. 源码索引

### SPF

- 作者方法和本地复现边界：`strategy/see_point_fly/REPRODUCTION_STATUS.md`
- Tello VLM 投点、深度和两个模式：
  `strategy/see_point_fly/upstream/src/spf/tello/action_projector.py`
- 作者原生持续循环：`strategy/see_point_fly/upstream/src/spf/tello/main.py:337`
- 本地作者代码适配 worker：`strategy/see_point_fly/worker/spf_worker.py:465`
- ActionPoint 到世界目标：
  `ros_nodes/mission/see_point_fly_bridge/scripts/see_point_fly_bridge.py:384`
- 本地多轮状态和操作员完成：
  `ros_nodes/mission/see_point_fly_bridge/scripts/spf_task_executor.py:23,165,189,269`
- SPF launch 与禁用 EGO 投影：`launch/bringup_see_point_fly.launch`

### SMPF

- 六个 mode 和主循环：`ros_nodes/mission/smpf_bridge/scripts/smpf_bridge.py:74,659,797`
- VLM 检测 schema 和提示词：`strategy/smpf/runtime/vision_detector.py:123-184`
- 长时域阶段分解：`strategy/smpf/runtime/task_stages.py:29-101`
- LLM guidepoint 和确定性回退：`strategy/smpf/runtime/model_planner.py:162-401`
- 三维记忆和目标身份：`strategy/smpf/runtime/scene_memory.py`、
  `strategy/smpf/runtime/target_identity.py`
- 走廊障碍筛选：`strategy/smpf/runtime/obstacle_relevance.py`
- 路径/终点验证：`strategy/smpf/runtime/geometry.py`、
  `strategy/smpf/runtime/goal_validation.py`
- Follow 驻点和终止规则：`strategy/smpf/runtime/follow_policy.py:39-199`
- waypoint 执行状态机：`strategy/smpf/runtime/execution.py:69-166`
- SMPF launch 参数：`ros_nodes/mission/smpf_bridge/launch/smpf_bridge.launch`

### 实验协议

- 固定真机任务：`strategy/smpf/experiments/spf_realworld_tasks.json`
- 作者仿真任务来源：`strategy/smpf/experiments/spf_simulation_tasks.json`
- 主对比配置：`strategy/smpf/experiments/comparison_profile.json`
- 操作说明：`strategy/smpf/experiments/README.md`
- 正式结果记录：`strategy/smpf/experiments/record_outcome.py`
- 配对完整性和汇总：`strategy/smpf/experiments/summarize_outcomes.py`

## 12. 最准确的一句话表述

SPF 是“用同一个 VLM 反应式图像投点闭环处理六类自然语言任务”；SMPF 是“先把可见
语义目标变成经过验证的三维目标/路线，再由 EGO 执行”，并对长时域、有限搜索和跟随
分别增加阶段状态、偏航扫描和单驻点重观察机制。两者当前都需要正式真机重复实验才能把
这些代码机制升级为经过验证的任务能力。
