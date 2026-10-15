# wpb_home_behaviors

启智机器人的物品检测和抓取行为。底盘、相机和机器人坐标变换由配套启动文件提供。

## 抓取控制

`src/grab.cpp` 实现 `wpb_home_grab_server` 的抓取流程。默认等待
`/wpb_home/behavior` 上的 `start grab` 指令；`auto_start=true` 时自动开始。
支持 `stop grab`、`grab stop`，并通过 `/wpb_home/grab_result` 发布
`grab done` 或 `grab failed`。完成、失败或空闲时释放底盘速度控制。

1. 每轮发送一次 `start objects`，取得一份有效物品坐标后发送 `stop objects`。
2. 比较物品坐标与对准目标；未对准时，发送 `pose_diff reset`，根据
   `/wpb_home/pose_diff` 以固定速度移动，停车后重新检测，直到对准。
3. 抬臂张爪，再清零相对里程计，按目标距离前进。
4. 停车夹取并抬起物品，再清零相对里程计，后退相同的目标距离。
5. 到位停车后发布完成结果。

所有等待均不阻塞回调，动作期间仍可处理停止指令。保留物品坐标校验、
机械臂高度检查、检测超时和退出停车，并检查位移反馈及移动超时。

## 默认参数

参数集中在 `src/grab.cpp` 开头的 `GrabParameters`，启动时也可以通过同名
ROS 参数覆盖。距离单位为米，速度单位为米/秒，时间单位为秒。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `align_x`、`align_y` | 1.0、0.0 | 物品目标为 `(align_x, align_y)` |
| `object_tolerance_x`、`object_tolerance_y` | 0.03、0.03 | 物品对准容差 |
| `move_tolerance_x`、`move_tolerance_y` | 0.02、0.01 | 位移到位容差 |
| `align_speed` | 0.05 | 对准时各轴速度 |
| `forward_speed` | 0.1 | 前进、后退速度 |
| `reset_wait`、`stop_wait` | 0.5、0.5 | 清零最短等待、到位停车等待 |
| `reset_tolerance` | 0.001 | 清零反馈容差，x/y 为米、theta 为弧度 |
| `forward_scale`、`forward_reference` | 0.7、0.7 | 前进距离为 `forward_scale * (object_x - forward_reference)` |
| `lift_offset`、`raise_height` | 0.04、0.05 | 初始抓取高度偏移、夹取后再抬高量 |
| `open_width`、`grip_width` | 0.15、0.045 | 张爪、夹取开度 |
| `lift_wait`、`grip_wait`、`raise_wait` | 18、5、5 | 抬臂、夹取、抬起等待 |
| `max_lift` | 1.2 | 机械臂最大目标高度 |
| `max_object_x`、`max_object_y` | 2.0、0.6 | 物品 x 上限和 y 绝对值上限 |
| `max_forward_distance` | 1.5 | 前进目标距离上限 |
| `object_timeout`、`detection_timeout` | 1、30 | 检测数据有效时间、每轮检测超时 |
| `motion_timeout`、`pose_timeout` | 30、1 | 单次清零或移动超时、移动中反馈超时 |

当 `object_x=1.0` 时，前进目标为 0.21 m，后退目标为 -0.21 m。
实际距离误差取决于底盘反馈和到位容差。

原比例对准和定时前后运动的参数已移除，包括 `align_gain`、`max_align_speed`、
`approach_speed`、`gripper_reach`、`approach_seconds_per_meter`、`max_approach_time`、
`retreat_time` 和 `retreat_speed`。
