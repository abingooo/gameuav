# SPF/SMPF 双 EGO 配置修改方案

## 1. 目标

为 SPF 和 SMPF 提供两种互斥的 EGO 运行配置：

```text
SMPF -> EGO mapped     -> 使用 RealSense 深度建图和局部轨迹避障 -> px4ctrl
SPF  -> EGO free_space -> 忽略场景障碍，仅做轨迹生成和动力学约束 -> px4ctrl
```

这里的“两种 EGO”不是维护两份 EGO 源代码，也不是同时启动两个规划节点，而是让同一个 EGO 二进制支持两个明确的启动 profile：

- `mapped`：保持当前 EGO 行为，融合深度并执行占据栅格碰撞检查。
- `free_space`：不把深度图、点云或历史占据栅格用于规划，将场景视为空旷空间。

两种 profile 应保持相同的里程计、目标坐标系、最大速度、最大加速度、最大跃度、B 样条参数、轨迹输出和 px4ctrl 控制链路。

## 2. 实验目的与结果归因

当前 SPF 和 SMPF 都通过 EGO 执行，因此整条飞行链的避障效果不能只归因于 SPF 作者的 `adaptive_mode` 或 SMPF 的上层规划。

采用双 profile 后：

- SPF 使用 `free_space` 时，EGO 不再根据场景障碍改变轨迹。SPF 成功避障主要反映 SPF 目标点选择和 `adaptive_mode` 的作用。
- SMPF 使用 `mapped` 时，最终避障效果包含 SMPF 上层规划和 EGO 局部深度避障的共同作用。
- 两者仍共享 EGO 的轨迹平滑、动力学可行性约束和 px4ctrl，避免重新引入“SPF 直接发位置给 px4ctrl、SMPF 使用 EGO”这种下游控制差异。

这是一组完整系统对比，不是相同下游规划器条件下的纯上层算法消融。论文和实验报告必须明确写出两个 profile，不能把 SMPF 的整链避障全部归因于 SMPF，也不能把 SPF 的整链避障全部归因于 `adaptive_mode`。

建议在主对比之外保留一组控制实验：

| 方法 | EGO profile | 用途 |
|---|---|---|
| SPF | `free_space` | 测 SPF 自身目标选择能否避障 |
| SMPF | `mapped` | 测 SMPF 完整系统表现 |
| SPF | `mapped` | 控制变量，量化 EGO 给 SPF 带来的增益 |
| SMPF | `free_space` | 消融实验，量化 EGO 局部地图对 SMPF 的贡献 |

## 3. 当前代码事实

当前 EGO 入口为：

- `launch/bringup_ego.launch`
- `ros_nodes/planning/ego_planner_stack/plan_manage/launch/single_run_in_exp.launch`
- `ros_nodes/planning/ego_planner_stack/plan_manage/launch/advanced_param_exp.xml`

当前深度输入固定为 `/camera/depth/image_rect_raw`，占据栅格由 `plan_env/src/grid_map.cpp` 建立，碰撞检查分布在 EGO 状态机、A* 和 B 样条优化器中。

`GridMap::hasDepthObservation()` 当前只有定义，没有被规划状态机用作“允许开始规划”的门槛。初始膨胀占据栅格为空，因此没有深度帧时理论上仍可生成轨迹。但是，仅把深度 remap 到不存在的话题不是可靠实现，原因包括：

- 代码仍然声称地图模块处于启用状态，实验元数据不明确。
- 深度丢失检查仍存在，后续修改或异常状态可能触发紧急停止。
- 运行时切换可能保留之前建立的占据栅格。
- 无法从日志可靠证明本次 SPF 实验确实没有使用障碍地图。

因此需要增加显式 profile/参数，而不是依赖“断开深度话题”这种隐式行为。

## 4. 推荐实现

### 4.1 新增统一 profile 参数

在 UAV 侧启动链增加：

```text
ego_profile:=mapped|free_space
```

参数逐层传递：

```text
config/modules/uav_agent.yaml
  -> launch/bringup_realflight.launch
  -> launch/bringup_ego.launch
  -> single_run_in_exp.launch
  -> advanced_param_exp.xml
  -> drone_0_ego_planner_node 私有参数
```

EGO 内部建议使用明确的布尔参数：

```text
grid_map/obstacle_mapping_enabled:=true|false
```

profile 到内部参数的映射应由 launch 完成：

```text
mapped     -> obstacle_mapping_enabled=true
free_space -> obstacle_mapping_enabled=false
```

不要让 SPF/SMPF 节点自行修改 EGO 的 ROS 参数。任务提交时修改参数无法清除节点内存里的旧地图，也容易在飞行中形成不完整切换。

### 4.2 `mapped` 行为

`mapped` 完全保持当前行为：

- 订阅并同步深度图和 VINS 里程计。
- 执行深度投影、射线更新、占据概率融合和障碍膨胀。
- B 样条优化、A* 搜索和轨迹安全检查读取占据栅格。
- 深度/里程计同步超时继续触发 EGO 的安全处理。

该 profile 用于 SMPF，也是普通真机飞行的默认配置。

### 4.3 `free_space` 行为

`free_space` 只关闭场景障碍地图和由它触发的局部避障，保留下列能力：

- VINS 世界系里程计输入。
- 世界系目标点输入。
- 全局/局部轨迹初始化。
- B 样条平滑。
- 最大速度、加速度、跃度和轨迹可行性约束。
- `/control/ego_position_cmd -> /control/position_cmd -> px4ctrl` 输出链。
- 地图的固定世界范围和飞行高度范围。

关闭时必须保证：

1. 深度图和点云不会写入占据栅格。
2. 原始占据栅格和膨胀占据栅格初始化并保持为空。
3. 场景占据检查不会触发重规划或紧急停止。
4. 深度丢失不会触发 `Depth Lost! EMERGENCY_STOP`。
5. VINS 丢失、非法目标、地图范围外目标和动力学不可行仍按原逻辑处理。

第 5 点很重要：`free_space` 是“忽略场景障碍”，不是关闭定位、范围和动力学约束。否则比较的不再只是局部避障能力。

### 4.4 建议的代码边界

优先在 `GridMap` 内统一实现禁用逻辑，避免在 A*、优化器和状态机的每一个碰撞调用点分别打补丁：

- `GridMap::initMap()` 读取 `grid_map/obstacle_mapping_enabled`。
- `false` 时不创建深度同步器和点云订阅器，仅保留里程计订阅。
- `getOccupancy()` 和 `getInflateOccupancy()` 对地图范围内的位置返回无占据。
- `getOdomDepthTimeout()` 在 `free_space` 下不报告深度超时。
- 节点启动时打印一次不可混淆的 profile 日志。

地图范围外目前由 `getInflateOccupancy()` 返回 `-1`，在布尔碰撞判断中会被视为占据。建议保留该行为，防止 `free_space` 轨迹飞出配置地图；不要为了忽略室内障碍而把所有越界位置也变成自由空间。

## 5. Profile 选择和切换

### 5.1 不同时运行两个 EGO 节点

两个无命名空间、无输出仲裁的 EGO 节点会竞争以下接口：

- `/planning/goal`
- EGO 内部 B 样条话题
- `/control/ego_position_cmd`

这会造成轨迹发布竞争、状态相互覆盖，并使实验结果无法解释。因此任意时刻只能运行一个 EGO profile。

### 5.2 切换必须重启 EGO 栈

建议实验流程：

```text
停止当前任务
-> 停止 EGO/轨迹服务器
-> 使用目标 profile 重新启动 EGO
-> 检查唯一发布者和 profile 状态
-> 再提交 SPF 或 SMPF 任务
```

重启的目的不仅是加载参数，也是彻底清除旧占据栅格、旧 B 样条和旧目标。第一版不要实现飞行中的热切换。

### 5.3 方法和 profile 的默认绑定

建议提供两个明确的 agent 启动入口或一个受约束参数：

```text
SPF  默认 ego_profile=free_space
SMPF 默认 ego_profile=mapped
```

控制实验允许人工覆盖，但每次覆盖必须写入实验日志。不能仅根据当前运行的是 SPF 还是 SMPF，在 EGO C++ 节点内部自动猜测 profile。

## 6. 日志和实验元数据

每次任务至少记录：

```json
{
  "method": "spf",
  "ego_profile": "free_space",
  "obstacle_mapping_enabled": false,
  "depth_topic_connected": false,
  "trajectory_owner": "ego",
  "control_route": "/control/ego_position -> EGO -> /control/ego_position_cmd -> /control/position_cmd -> px4ctrl"
}
```

还应记录：

- EGO 节点启动时间和配置摘要。
- 深度输入帧数、点云输入帧数和地图占据体素数。
- 目标点、实际轨迹、规划耗时、重规划次数和任务结果。
- 是否发生地图碰撞重规划、动力学重定时或紧急停止。
- SPF 的 `operational_mode`，例如 `adaptive_mode`。

`free_space` 实验的验收条件之一应是：深度/点云融合计数为 0、占据体素数为 0，而不是仅凭 RViz 中“看不到点云”判断。

## 7. 验证顺序

### 7.1 静态检查和单元测试

- launch 参数能够从 agent 配置传到 EGO 节点。
- 未提供 profile 时默认 `mapped`，避免改变现有真机行为。
- 非法 profile 直接启动失败，不允许静默回退。
- `mapped` 下地图查询行为与修改前一致。
- `free_space` 下地图内障碍查询始终为空，地图外仍被拒绝。
- `free_space` 下深度超时不会触发紧急停止。

### 7.2 无桨/无电池链路验证

- 分别启动两个 profile，确认 ROS 图中只有一个 EGO 规划节点和一个轨迹服务器。
- 对同一个世界系目标发布 `/control/ego_position`。
- 两个 profile 都应产生 `/control/ego_position_cmd`。
- `mapped` 应产生占据地图；`free_space` 占据体素数必须保持为 0。
- 切换 profile 后确认上一轮地图和轨迹不存在。

### 7.3 受控障碍验证

在相同起点、终点、速度和障碍布局下：

- `mapped` 轨迹应根据深度障碍偏转或拒绝不可行路径。
- `free_space` 轨迹不应因该障碍偏转，但仍应满足速度和加速度限制。
- SPF 的目标点必须原样进入 `free_space` EGO，不能由额外的安全过滤器偷偷修改。

先用手持、仿真或不装桨条件验证输出轨迹。`free_space` 确认有效后，其真机行为就是可能直接撞向障碍，不能依靠 EGO 兜底。

## 8. 建议实施顺序

1. 增加 `GridMap` 显式 `obstacle_mapping_enabled` 参数和测试。
2. 增加 `mapped/free_space` launch profile 传递与非法值检查。
3. 将 agent 中现有 EGO 默认值保持为 `mapped`。
4. 增加 SPF-free-space 和 SMPF-mapped 的明确启动入口。
5. 给实验日志增加 profile、地图输入计数和占据体素计数。
6. 完成无桨链路验证和 RViz 对照验证。
7. 最后才进行隔离场地的低速真机测试。

## 9. 完成判据

修改完成必须同时满足：

- SPF 与 SMPF 均继续通过 EGO 向 px4ctrl 输出轨迹指令。
- SPF 的 `free_space` profile 不读取或使用任何场景障碍地图。
- SMPF 的 `mapped` profile 与当前 EGO 深度避障行为一致。
- 两个 profile 不同时发布控制轨迹。
- profile 切换不会继承旧地图或旧轨迹。
- 实验日志能够证明每次运行实际使用了哪个 profile。
- 现有自动化测试通过，并新增 profile 行为测试。
