# ROS 局部规划器配置与 DWA 算法通俗讲解

本文面向第一次接触 ROS 导航的同学，目标是用大白话把两个事情讲清楚：

1. **DWA 算法到底在干什么？**
2. **怎么在 ROS（move_base）里把 DWA 局部规划器配置起来？**

---

## 1. 先搞清楚"全局规划"和"局部规划"的区别

ROS 的导航栈 `move_base` 把机器人"怎么走过去"这件事拆成两层：

| 层级       | 角色            | 比喻                                                  |
| ---------- | --------------- | ----------------------------------------------------- |
| 全局规划器 | global_planner  | **百度地图**：从家到公司大致走哪条路（A* / Dijkstra） |
| 局部规划器 | local_planner   | **真实开车的你**：眼前有车有人怎么避，怎么打方向盘    |

- 全局规划：在已知静态地图上算一条从起点到终点的最短路径，不太管动态障碍。
- 局部规划：每隔几十毫秒看一次激光雷达/深度相机，实时决定**下一秒油门/方向盘怎么打**，避开突然出现的人和障碍。

**DWA 就是最常用的局部规划算法之一**。

---

## 2. DWA 算法是什么 —— 一句话版本

> DWA（Dynamic Window Approach，动态窗口法）：
> **"在我下一秒物理上能做到的所有(线速度, 角速度)组合里，挑一个最不撞墙、最朝目标、走得最快的去执行。"**

听起来是不是和你开车很像？

---

## 3. DWA 算法详细但通俗的过程

每一个控制周期（比如 50 ms）DWA 都干这 4 件事：

### 步骤 1：列出"我能做到的速度"——动态窗口

机器人有最大速度、最大加速度。所以下一秒的速度不可能想多大就多大。

举例：
- 当前线速度 0.3 m/s，最大加速度 0.5 m/s²，控制周期 0.1 s
- 那么下一周期能做到的线速度只能是 `0.3 ± 0.5×0.1 = 0.25 ~ 0.35 m/s`
- 角速度同理，得到一个范围

把所有"可行的(v, ω)对"组成一个**二维窗口**，这就是"动态窗口"的意思。

### 步骤 2：在这个窗口里采样一堆候选速度

在窗口内按一定密度采样，比如 v 取 20 个值，ω 取 40 个值，得到 800 组候选 `(v, ω)`。

### 步骤 3：每组速度都"假装走几秒看看"

对每一组 `(v, ω)`：

- 假设接下来 1~3 秒一直保持这个速度不变；
- 用运动学公式画出一条预测轨迹（一段圆弧）；
- 再去 costmap（代价地图）里查这条轨迹会不会碰到障碍。

### 步骤 4：给每条轨迹打分，挑最高的

DWA 的打分函数大概长这样：

```
score = α·朝向目标的得分
      + β·避障得分（离障碍越远越好）
      + γ·速度得分（越快越好）
```

- 撞墙的轨迹直接淘汰；
- 朝着全局路径前进的得分高；
- 在空旷区域跑得快的得分高。

最终得分最高的那一组 `(v, ω)` 就是这一帧发给底盘的指令。然后下一周期重新算一次，循环往复。

### 用一张表总结 DWA

| 关键词       | 含义                                                       |
| ------------ | ---------------------------------------------------------- |
| Dynamic      | 考虑机器人**动力学约束**（最大加减速）                     |
| Window       | 当前速度 ± 加减速范围内的"可达速度窗口"                    |
| Approach     | 在窗口内**采样 + 模拟 + 评分** 选最优                      |

---

## 4. ROS 里怎么把 DWA 局部规划器跑起来

ROS1 (Melodic/Noetic) 下，DWA 局部规划器的包是 `dwa_local_planner`。整体流程：

```
传感器 → /scan、/odom、/tf
              ↓
           move_base
       ┌─────┴─────┐
   全局规划器     局部规划器(DWA)
       │              │
   global_costmap  local_costmap
              ↓
        /cmd_vel → 底盘
```

### 4.1 安装

```bash
sudo apt install ros-${ROS_DISTRO}-navigation
```

`navigation` 元包里已经包含 `move_base`、`dwa_local_planner`、`costmap_2d` 等。

### 4.2 准备好这几样东西

| 输入            | 谁提供                  | 说明                       |
| --------------- | ----------------------- | -------------------------- |
| `/tf`           | robot_state_publisher 等 | base_link ↔ odom ↔ map     |
| `/odom`         | 底盘驱动节点            | 里程计（线/角速度+位姿）   |
| `/scan` 或点云  | 激光雷达驱动            | 障碍来源                   |
| `map` (可选)    | map_server / SLAM       | 用于全局规划               |

底盘需要订阅 `/cmd_vel` 并按 `geometry_msgs/Twist` 执行（线速度 + 角速度）。

### 4.3 5 个核心配置文件

通常在你自己的 `xxx_navigation` 包的 `config/` 下放：

```
config/
├── costmap_common_params.yaml   # 全局/局部代价地图共用参数
├── global_costmap_params.yaml   # 全局地图（一般静态、覆盖整张地图）
├── local_costmap_params.yaml    # 局部地图（小窗口、跟随机器人移动）
├── base_local_planner_params.yaml  # ★ DWA 参数
└── move_base_params.yaml        # 选哪个全局/局部规划器、频率等
```

### 4.4 `move_base` 启动文件示例

```xml
<launch>
  <node pkg="move_base" type="move_base" name="move_base" output="screen">
    <!-- 选用 DWA 作为局部规划器 -->
    <param name="base_local_planner"
           value="dwa_local_planner/DWAPlannerROS"/>
    <param name="base_global_planner"
           value="global_planner/GlobalPlanner"/>

    <rosparam file="$(find my_nav)/config/costmap_common_params.yaml"
              command="load" ns="global_costmap"/>
    <rosparam file="$(find my_nav)/config/costmap_common_params.yaml"
              command="load" ns="local_costmap"/>
    <rosparam file="$(find my_nav)/config/global_costmap_params.yaml"
              command="load"/>
    <rosparam file="$(find my_nav)/config/local_costmap_params.yaml"
              command="load"/>
    <rosparam file="$(find my_nav)/config/base_local_planner_params.yaml"
              command="load"/>
  </node>
</launch>
```

### 4.5 `costmap_common_params.yaml`

```yaml
robot_radius: 0.20            # 机器人外接圆半径(米)；非圆机器人用 footprint
inflation_radius: 0.30        # 障碍膨胀半径，越大越保守
cost_scaling_factor: 5.0

obstacle_range: 2.5           # 多远以内的障碍计入地图
raytrace_range: 3.0           # 清除多远的"过期"障碍

observation_sources: laser_scan_sensor
laser_scan_sensor: {
  sensor_frame: laser_link,
  data_type: LaserScan,
  topic: /scan,
  marking: true,
  clearing: true
}
```

### 4.6 `local_costmap_params.yaml`

```yaml
local_costmap:
  global_frame: odom        # 局部图常用 odom 坐标系，避免抖动
  robot_base_frame: base_link
  update_frequency: 10.0
  publish_frequency: 5.0
  width: 4.0                # 局部窗口大小(米)
  height: 4.0
  resolution: 0.05
  rolling_window: true      # 让局部图跟着机器人滑动
  static_map: false
```

### 4.7 `base_local_planner_params.yaml`（**重点：DWA 调参**）

```yaml
DWAPlannerROS:
  # ---- 速度/加速度限制 ----
  max_vel_x: 0.5            # 前向最大线速度 m/s
  min_vel_x: 0.0            # 不允许后退就设 0；允许后退用负值
  max_vel_y: 0.0            # 差速/阿克曼车保持 0；全向车再开
  min_vel_y: 0.0
  max_vel_trans: 0.5
  min_vel_trans: 0.05
  max_vel_theta: 1.5        # 最大角速度 rad/s
  min_vel_theta: 0.4
  acc_lim_x: 1.0            # 线加速度上限
  acc_lim_y: 0.0
  acc_lim_theta: 2.0        # 角加速度上限

  # ---- 到点判定 ----
  xy_goal_tolerance: 0.10   # 距离目标点 10cm 视为到达
  yaw_goal_tolerance: 0.15  # 朝向误差容忍 ~8.6°
  latch_xy_goal_tolerance: false

  # ---- 轨迹采样/模拟 ----
  sim_time: 1.5             # 每条候选轨迹"假装走 1.5 秒"
  vx_samples: 20            # 线速度采样数
  vy_samples: 1             # 差速车 1 即可
  vth_samples: 40           # 角速度采样数
  controller_frequency: 10  # 局部规划频率 Hz

  # ---- 评分权重(打分函数里 α/β/γ) ----
  path_distance_bias: 32.0  # 越大越贴着全局路径走
  goal_distance_bias: 24.0  # 越大越急着冲向目标
  occdist_scale: 0.02       # 越大越远离障碍（更怂）

  # ---- 防卡死 ----
  oscillation_reset_dist: 0.05
  prune_plan: true
```

### 4.8 启动顺序

```bash
# 终端 1：底盘驱动（发布 /odom，订阅 /cmd_vel）
roslaunch my_chassis bringup.launch
# 终端 2：激光雷达
roslaunch my_lidar lidar.launch
# 终端 3：地图 + AMCL（已知地图）或 SLAM
roslaunch my_nav amcl.launch
# 终端 4：move_base
roslaunch my_nav move_base.launch
# 终端 5：可视化
rviz
```

在 RViz 里用 **2D Nav Goal** 点一个目标，机器人就会用 DWA 局部规划器一边避障一边走过去。

---

## 5. 调参速查（出问题时先看这里）

| 现象                     | 该调哪个参数                                                        |
| ------------------------ | ------------------------------------------------------------------- |
| 机器人贴墙太近、容易蹭   | 加大 `inflation_radius`、`occdist_scale`                            |
| 老是绕远路、不走全局路径 | 加大 `path_distance_bias`                                           |
| 在目标附近反复打转       | 加大 `xy_goal_tolerance`、`yaw_goal_tolerance`                      |
| 起步太肉/刹车太硬        | 调 `acc_lim_x`、`acc_lim_theta`                                     |
| 在窄通道里抖动停住       | 减小 `vx_samples`、加大 `sim_time`，或减小 `inflation_radius`       |
| CPU 占用高               | 减小 `vx_samples`、`vth_samples`、`controller_frequency`            |
| 走得太慢                 | 加大 `max_vel_x`，并对应调 `acc_lim_x`                              |
| 突然冲出去撞东西         | 检查 `/scan`、`/tf`、`/odom` 频率与时间戳是否正常                   |

---

## 6. DWA 不是唯一选择

| 局部规划器                  | 适用                                                |
| --------------------------- | --------------------------------------------------- |
| `dwa_local_planner`         | 通用，差速/全向都能用，调参直观                     |
| `base_local_planner` (TR)   | 老牌 Trajectory Rollout，思路类似 DWA               |
| `teb_local_planner`         | **TEB**：弹性带优化，曲线漂亮，适合阿克曼/狭窄环境  |
| `mpc_local_planner`         | 模型预测控制，更平滑但参数多、计算重                |

如果你的车是阿克曼（不能原地转）或对路径平滑度要求高，建议试试 **TEB**；普通差速/麦轮小车，**DWA 通常够用**。

---

## 7. 一句话记住 DWA

> **"在我能踩到的油门和能打到的方向盘范围内，模拟一堆未来 1 秒会走出的弧线，挑那条不撞、最朝目标、最快的执行——每 100 毫秒重做一次。"**

这就是 DWA。
