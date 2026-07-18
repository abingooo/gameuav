# fast_drone_250.yaml 参数说明

配置文件：`/home/uav/Desktop/uav_project/gameuav/ros_nodes/state_estimation/VINS-Fusion/config/fast_drone_250.yaml`

> 说明基于当前仓库代码实现（`vins_estimator` + `loop_fusion`）。

## 1. 总览

- 这是一个 **双目 + IMU** 的 VINS 配置（`imu: 1`, `num_of_cam: 2`）。
- `vins_node` 会读取大部分参数。
- 回环相关参数只在 `loop_fusion_node` 生效。
- 当前 `fast_drone_250.launch` 中 `loop_fusion` 节点是注释状态（默认不启用）。

## 2. 参数逐项说明

## 2.1 传感器与话题

| 参数 | 当前值 | 作用 | 备注 |
|---|---:|---|---|
| `imu` | `1` | 是否使用 IMU。`1`=启用，`0`=纯视觉。 | 当 `imu=0` 时，代码会强制 `estimate_extrinsic=0`、`estimate_td=0`。 |
| `num_of_cam` | `2` | 相机数量。支持 `1` 或 `2`。 | 设为 `2` 时启用双目流程；非 1/2 会直接触发断言退出。 |
| `imu_topic` | `/mavros/imu/data` | IMU 订阅话题。 | 仅 `imu=1` 时使用；当前 PX4/MAVROS 实测 `data_raw` 无数据。 |
| `image0_topic` | `/camera/infra1/image_rect_raw` | 左目图像话题。 | VINS 与（可选）回环模块都会用。 |
| `image1_topic` | `/camera/infra2/image_rect_raw` | 右目图像话题。 | 仅双目（`num_of_cam=2`）时使用。 |

## 2.2 输出与标定文件

| 参数 | 当前值 | 作用 | 备注 |
|---|---|---|---|
| `output_path` | `/home/uav/Desktop/uav_project/gameuav/runtime/vins_output` | VINS/回环结果输出目录。 | VINS 会写 `stamped_traj_estimate.txt`，回环会写 `vio_loop.txt`。目录需提前存在。 |
| `save_result` | `0` | 是否保存 VINS txt 调试结果。 | `0`=关闭每帧写 `stamped_traj_estimate.txt` / `bias.txt`，减轻无人机端 I/O；`1`=打开，便于离线分析。 |
| `cam0_calib` | `left.yaml` | 左目内参文件。 | 路径按“**相对当前 config 文件目录**”拼接。 |
| `cam1_calib` | `right.yaml` | 右目内参文件。 | 同上。 |
| `image_width` | `640` | 图像宽度配置值。 | 在 VINS 主流程里主要用于记录；回环模块会用它做可视化投影。 |
| `image_height` | `480` | 图像高度配置值。 | 同上。 |

## 2.3 IMU-相机外参

| 参数 | 当前值 | 作用 | 备注 |
|---|---|---|---|
| `estimate_extrinsic` | `0` | 外参处理模式。 | `0`=固定外参；`1`=在初值附近优化；代码还支持 `2`=无先验、从单位阵初始化。当前实机调试中固定外参，避免静止/激励不足时在线外参漂移。 |
| `body_T_cam0` | 4x4矩阵 | `body(imu) <- cam0` 的外参（旋转+平移）。 | 作为左目初始外参。平移约 `(0.0013, -0.0041, 0.0091) m`。 |
| `body_T_cam1` | 4x4矩阵 | `body(imu) <- cam1` 的外参（旋转+平移）。 | 作为右目外参。平移约 `(0.0040, -0.0558, 0.0056) m`。 |

补充：文件里 `# 20260123 online` 那组矩阵是注释状态，**当前不会被读取**；真正生效的是 `# offline_param` 下方两组。

## 2.4 线程与特征跟踪

| 参数 | 当前值 | 作用 | 备注 |
|---|---:|---|---|
| `multiple_thread` | `1` | 是否多线程处理。 | 在这份实现里，开启后会启动后台处理线程，并且只将每 2 帧中的 1 帧送入后端（`inputImageCnt % 2 == 0`）。 |
| `max_cnt` | `120` | 每帧最多跟踪/维持的特征点数量。 | 原实机值 150；当前降到 120 以减轻 VINS 特征跟踪/后端约束负载，若动态飞行鲁棒性不足可回调。 |
| `min_dist` | `30` | 新特征点之间最小像素间距。 | 越大点更稀疏；越小点更密集。 |
| `freq` | `10` | 期望发布频率（历史字段）。 | **当前仓库代码未读取该参数**，属于未生效字段。 |
| `F_threshold` | `1.0` | 基础矩阵 RANSAC 阈值（像素）。 | 当前 `rejectWithF()` 调用被注释，**此参数目前不生效**。 |
| `show_track` | `0` | 是否发布跟踪可视化图。 | 发布到 `image_track`。实机默认关闭，避免无人机端在无订阅者时仍绘制/发布 30Hz 调试图像。 |
| `flow_back` | `1` | 是否启用前后向光流一致性检查。 | 可提高鲁棒性，但会增加跟踪开销。 |

## 2.5 后端优化

| 参数 | 当前值 | 作用 | 备注 |
|---|---:|---|---|
| `max_solver_time` | `0.04` | 单次优化最大求解时间。 | 传给 Ceres 的是 `max_solver_time_in_seconds`，即 **单位是秒**（0.04s=40ms）。 |
| `max_num_iterations` | `8` | Ceres 最大迭代次数。 | 次数越大收敛机会更高，但实时性更差。 |
| `keyframe_parallax` | `10.0` | 关键帧判定的视差阈值（像素）。 | 代码内部会除以 `FOCAL_LENGTH=460` 转成归一化阈值。 |

## 2.6 IMU 噪声与重力

| 参数 | 当前值 | 作用 | 备注 |
|---|---:|---|---|
| `acc_n` | `0.1` | 加速度计测量噪声标准差。 | 用于预积分噪声协方差。 |
| `gyr_n` | `0.01` | 陀螺仪测量噪声标准差。 | 同上。 |
| `acc_w` | `0.001` | 加速度计零偏随机游走标准差。 | 同上。 |
| `gyr_w` | `0.0001` | 陀螺仪零偏随机游走标准差。 | 同上。 |
| `g_norm` | `9.805` | 重力模长。 | 直接设置重力向量 `G=(0,0,g_norm)`。 |

## 2.7 时间偏移（相机-IMU不同步）

| 参数 | 当前值 | 作用 | 备注 |
|---|---:|---|---|
| `estimate_td` | `0` | 是否在线估计时间偏移 `td`。 | 当前固定 `td`，避免静止/激励不足时在线时间偏移把 VINS 带偏。 |
| `td` | `-0.05` | 初始时间偏移（秒）。 | 在代码中按 `image_time + td` 与 IMU 对齐。 |

## 2.8 回环相关（仅 loop_fusion 节点）

| 参数 | 当前值 | 作用 | 备注 |
|---|---|---|---|
| `load_previous_pose_graph` | `0` | 是否启动时加载历史位姿图。 | `1` 时从 `pose_graph_save_path` 读取。 |
| `pose_graph_save_path` | `/home/uav/Desktop/uav_project/gameuav/runtime/pose_graph/` | 位姿图保存/加载目录。 | 需有读写权限且目录存在。 |
| `save_image` | `1` | 回环中是否保留/保存关键帧图像。 | 关闭可省内存/磁盘，但调试可视化会减少。 |

## 3. 当前配置里的“隐含坑位”

- `freq` 当前未被任何代码读取，改它不会改变系统行为。
- `F_threshold` 当前未生效（RANSAC剔错调用被注释）。
- `max_solver_time` 注释写了 `ms`，但实现按 **秒** 使用。
- `cam0_calib/cam1_calib` 是按 config 目录拼相对路径，不建议直接写绝对路径。
- 回环参数只有在启动 `loop_fusion_node` 后才生效；当前 launch 默认未启用回环节点。
