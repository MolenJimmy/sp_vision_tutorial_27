# 算法组导航方向大作业&#x20;

## 1. 项目概览

本工程是一个 ROS2 Humble 下的"一次点击导航"系统：行为树下发目标 → A\* 规划全局路径 → 自研控制器插件跟踪路径 → 全向底盘仿真器执行。

在发放代码基础上完成了三部分工作：

| 部分  | 内容                                                | 涉及文件                                                    |
| --- | ------------------------------------------------- | ------------------------------------------------------- |
| 必做① | 补全全系统参数，接通所有话题/插件链路                               | `sp_nav_bringup/config/nav_params.yaml`                 |
| 必做② | 实现路径跟踪控制器插件（PD + 预瞄 + 终点减速 + 加速度限幅）               | `sp_controller_server/plugins/pid_controller.{hpp,cpp}` |
| 必做③ | RViz 可视化（代价地图 / 全局路径 / 机器人位姿 / 目标点）、git 管理        | `sp_nav_bringup/rviz/rviz.rviz`、仓库 commit 记录            |
| 加分项 | A\* 轨迹平滑：视线剪枝（shortcut）+ 切角圆化（Chaikin），并做了几何级安全校验 | `sp_global_planner/plugins/astar_planner.{hpp,cpp}`     |

## 2. 系统架构与数据流

```
sp_decision (决策BT 10Hz)
    │ action: navigate_to_pose
    ▼
nav_interface_node (导航BT)
    │ TF 查位姿 ──► /make_plan 服务 ──► planner_server (A*)
    │                                     ▲ 融合 /global_costmap + /local_costmap
    │ /set_control_enable(true)
    ▼
controller_server (50Hz 控制环)
    │ 路径 + 位姿 + 速度 ──► PidController 插件
    ▼ /sentry/cmd_vel (Twist, base_link 系)
sp_nav_sim (运动学积分 → odom/TF) ──► 闭环
```

三条"总线"：TF（位姿）、OccupancyGrid 话题（地图）、`/sentry/cmd_vel`（指令）。两层行为树之间通过 `NavigateToPose` action 通信，规划器与控制器之间通过 `/global_path` 话题衔接。

## 3. 必做工作 ①：参数补全（nav\_params.yaml）

发放的 yaml 大部分字段为空，工程无法启动（各节点 `require_param` 读不到参数会直接抛异常）。

### 3.1 话题名&#x20;

仿真器与 esdf 发布器的话题名在源码中写死、且不允许修改仿真器，只能让 yaml 迁就：

| 参数                                   | 值                        | 依据        |
| ------------------------------------ | ------------------------ | --------- |
| `controller_server.odom_topic`       | `/Odometry`              | 仿真器固定发布   |
| `controller_server.odom_frame_id`    | `lidar_odom`             | 仿真器 TF 命名 |
| `controller_server.cmd_vel_topic`    | `/sentry/cmd_vel`        | 仿真器固定订阅   |
| `planner_server.local_costmap_topic` | `/local_costmap/costmap` | 仿真器固定发布   |

### 3.2 costmap 选择

最初给 `planner_server.costmap_topic` 填了 `/esdf_costmap`，结果**路径穿墙**。排查源码后发现三个话题的编码完全不同：

- `/esdf_costmap` 的格子值 = **到障碍的距离 × 100**：墙 ≈ 0，开阔地 ≈ 127 —— 数值语义与 A\* 的"≥ lethal\_cost 视为墙"**正好相反**，导致"墙可走、开阔地是墙"，A\* 专门挑贴墙的路走

- `/global_costmap` 的格子值才是"危险程度"：`robot_radius+margin` 内 = 100（墙），0.30~~0.70m 为 1~~99 惩罚，之外为 0

**修复**：`costmap_topic: "/global_costmap"`，与 `lethal_cost: 100` 咬合。

### 3.3 插件注册名

`plugin_type` 填 plugin\_description.xml 中的 `name` 字段（查找名）：

- 规划器：`sp_global_planner/AStarPlanner`

- 控制器：`PidController`

且 `plugin_name`与参数块名必须一致：`plugin_name: "AStar"` ↔ `AStar:` 块；`PidController:` 同理。

### 3.4 控制器参数

```yaml
PidController:
  base_frame_id: "base_link"
  kp_v: 1.5            # 速度比例增益
  v_max: 2.0           # 与仿真器 v_max 对齐
  lookahead_distance: 0.5
  arrive_tol: 0.05     # 比 BT 判到的 0.3 更严，保证判定到达时位置准确
  kd_v: 0.8            # 横向阻尼
  decel_dist: 1.0      # 终点减速区
  a_max: 2.0           # 与仿真器 a_max 对齐
```

## 4. 必做工作 ②：控制器插件（PidController）

### 4.1 总体思路

在 `computeVelocityCommands(pose, velocity)` 中按五步流计算 50Hz 速度指令：

```
找最近点 → 沿路径取预瞄点 → 转车体系 → PD 控制 → 三级限幅（v_max → 终点减速 → 加速度限幅）
```

### 4.2 各步说明

**① 最近点搜索**：遍历路径求距车最近的点下标，确定在路径上的进度。

**② 预瞄点**：从最近点沿路径累计弧长，进一步考虑车速越快预瞄距离越远，前进 `lookahead_distance + 0.5 * last_speed_` 处作为临时目标。直接追最近点会让车贴着已走过的路径慢慢蹭，预瞄让车"看向远方"，转弯提前转向、直线不易画龙。接近终点时弧长不足，目标自动落在终点。

**③ 转车体系**：从位姿四元数提取偏航角

```
yaw = atan2(2(wz+xy), 1−2(y²+z²))
```

将 map 系偏差向量按 yaw 旋转，得到偏差在车体系分量：

```
ex =  cos(yaw)·mx + sin(yaw)·my    （车头方向）
ey = −sin(yaw)·mx + cos(yaw)·my    （车体左侧）
```

**④ PD 控制**（横向加阻尼）：

```
vx = kp_v · ex
vy = kp_v · ey + kd_v · (ey − last_ey)
```

**⑤ 三级限幅**（依次收紧）：

1. 速度上限 `v_max`（2.0，与仿真器一致）
2. **终点减速段**：距真终点 < `decel_dist` 时，允许速度线性收缩到 `v_max·dist/decel_dist`（保底 0.1 m/s）——保证准确到达
3. **加速度限幅**：每拍速度变化 ≤ `a_max × dt`，输出平滑无阶跃

\*\*       到达判断**用真终点距离 <** **`arrive_tol`** **时输出零速并**清零全部状态\*\*。

### 4.3 状态管理（容易出错的细节）

PD 与限幅都依赖跨拍记忆，两个时机必须清零，否则旧目标的记忆会污染新导航：

- 判定到达返回零速时：同样清零

- `setPlan()` 收到新路径不应该清零，`planner_server`一定时间内会规划新路线，在此处重置会导致车速停留在一个较小的区间，导致运行时间变长

### 4.4 输出坐标系

`cmd_vel.header.frame_id = "base_link"`：仿真底盘按车体系解释指令（x 车头、y 车左）。

## 5. 加分项：A\* 轨迹平滑（sp\_global\_planner）

A\* 输出是栅格阶梯折线：直线上数百个冗余点、拐角是 90° 直角。平滑分两刀 + 一道安全闸，全部在 `createPlan` 出路径后处理：

### 5.1 视线剪枝（shortcut）

直觉：两点之间直线无遮挡，中间点全是废点。贪心实现——从当前锚点向路径最远端倒序尝试，找到"能直连的最远点"就跳过去：

```
501 点锯齿路径 → 4~10 个锚点，直线彻底笔直
```

### 5.2 切角圆化（Chaikin）

对每段相邻点对 (A,B) 生成两个 1/4、3/4 处的新点，端点保留，迭代 2 轮：

```
P = 0.75·A + 0.25·B,  Q = 0.25·A + 0.75·B
```

尖角被逐步削成圆弧。选择 Chaikin 而非样条插值的原因：**新点永远落在原线段内，曲线被原折线包住，安全余量不会变小**——不需要额外的碰撞检查就天然安全。

### 5.3 几何级安全校验（lineFree 通道检查）

初版剪枝曾出现"抄近道贴墙"。最终版本采用几何精确判据，而非代价阈值：

- 对线段附近穷举墙格（lethal 格），计算每个墙格中心到线段的**解析距离**（点到线段投影并 clamp 到 \[0,1]）

- 距离 ≤ `reach = res + res·√2/2` 判为不可通行——外扩量覆盖栅格离散化误差（正方形格子按中心判距，角部最多比中心近半个对角线）

- 车半径安全由地图膨胀（`robot_radius + margin`）承担，通道检查只补离散误差，职责单一

- **Chaikin 每轮结果逐线段校验，不通过则回退该轮并告警**——平滑永不破坏安全性

### 5.4 效果

- 路径点数：数百 → 数十

- 直线段笔直小车到达终点的耗时变短

- 转弯可能存在撞墙的情况（根据终点的设置情况不同），可能原因是对路线的裁剪会把一段曲线拉直，导致直线通过小车与墙面的间隔太短而撞墙，还需要充分结合代价地图来优化规划算法才能够投入使用。

## 6. 可视化与代码管理

- RViz 展示：全局代价地图（`/global_costmap`）、全局路径（`/global_path`）、机器人位姿（TF）、目标点（`/goal_pose` / plan markers）

- `.gitignore` 忽略 `build/ install/ log/`；参数、控制器、平滑分小 commit 提交，见仓库历史

## 7. 复现步骤

```bash
colcon build
source install/setup.bash
ros2 launch sp_nav_sim sim_robot.launch.py        # 终端1：仿真器
ros2 launch sp_nav_bringup sp_nav.launch.py       # 终端2：导航全家桶
# RViz 中用 2D Goal Pose 对 (14.1, 14.1) 点击一次即开始导航
```

