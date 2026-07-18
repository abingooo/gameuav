已迁移 ROS 模块

  | 模块 | 外侧输入 | 外侧输出 | 说明 |
  |---|---|---|---|
  | ros_nodes/perception/realsense-ros | RealSense USB 设备<br>launch 参数<br>可选 odom_in | camera/infra1/
  image_rect_raw<br>camera/infra2/image_rect_raw<br>camera/depth/image_rect_raw<br>camera/color/image_raw<br>camera/*/
  camera_info<br>可选 IMU / 点云 / TF | 负责传感器数据采集 |
  | ros_nodes/state_estimation/VINS-Fusion | mavros/imu/data<br>camera/infra1/image_rect_raw<br>camera/infra2/
  image_rect_raw<br>vins_restart<br>vins_imu_switch<br>vins_cam_switch | vins_fusion/imu_propagate<br>vins_fusion/
  odometry<br>vins_fusion/path<br>vins_fusion/extrinsic<br>image_track<br>point_cloud | 核心输出是里程计，供规划和控制使
  用 |
  | ros_nodes/planning/ego_planner_stack | vins_fusion/imu_propagate<br>camera/depth/image_rect_raw<br>vins_fusion/
  extrinsic<br>/planning/goal<br>/planning/goal_yaw_deg<br>broadcast_bspline | position_cmd<br>drone_0_planning/
  bspline<br>drone_0_planning/data_display<br>grid_map/occupancy | 输入目标点和感知状态，输出轨
  迹控制指令 |
  | ros_nodes/control/px4ctrl | vins_fusion/imu_propagate<br>position_cmd<br>px4ctrl/takeoff_land<br>px4ctrl/
  hover_yaw_cmd<br>mavros/state<br>mavros/imu/data<br>mavros/battery<br>mavros/rc/in | mavros/setpoint_raw/
  attitude<br>mavros/setpoint_position/local<br>traj_start_trigger<br>debugPx4ctrl<br>调用 mavros/set_mode / mavros/cmd/
  arming | 连接规划输出和 PX4 飞控 |
  | ros_nodes/actuation/led/tiplight | /actuation/tiplight_cmd<br>std_msgs/String<br>串口参数 port/baud | /status/tiplight<br>std_msgs/String<br>串口命令到 LED
  控制板<br>默认 /dev/gameuav_tiplight | 非飞行执行器，属于 actuation |

  架构层模块

  | 模块 | 外侧输入 | 外侧输出 | 说明 |
  |---|---|---|---|
  | ros_nodes/mission | GCS 任务命令<br>策略输出<br>当前无人机状态 | 任务阶段<br>目标点<br>起飞/降落/暂停/恢复命令 | 任
  务状态机 |
  | ros_nodes/safety | 电量<br>定位健康<br>通信心跳<br>飞控状态<br>越界/碰撞风险 | 悬停<br>返航<br>降落<br>急停<br>告警
  | 本机独立安全保护 |
  | agent/uav_agent | GCS 模块管理命令 | 启停 ROS 模块<br>返回 ACK / 状态 / 错误码 | 不是任意 shell，是受控命令执行器 |
  | gateway/ros_to_net_gateway | 本机 ROS 状态 topic | TCP/UDP 网络状态消息 | ROS 到网络 |
  | gateway/net_to_ros_gateway | GCS / 其他无人机网络消息 | ROS topic / service / action | 网络到 ROS |
  | comm | gateway 或 agent 传入的结构化消息 | UDP 高频消息<br>TCP 可靠消息<br>心跳<br>校验 | 纯通信层 |
  | gcs | 无人机状态<br>日志<br>告警<br>轨迹 | 任务下发<br>模块启停<br>参数修改<br>人工接管 | 地面站 |
  | strategy | 多机状态<br>任务目标<br>约束条件 | 航点<br>编队目标<br>策略模式 | ADV/MPC/规则/编队插件 |
  | config | 人工配置文件 | 被各模块读取 | 机队、网络、话题、坐标系、模块白名单 |
  | ros_nodes/common | 被其他包依赖 | msg / 工具库 / CMake 支持 | 公共依赖，不作为业务模块运行 |
