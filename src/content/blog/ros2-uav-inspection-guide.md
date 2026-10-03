---
title: "ROS 2 两栖无人机全域巡检实战：从飞机飞走"到全流程一次通过"
date: 2026-10-03
description: "一场从零到一的仿真项目复盘：环境踩坑、控制安全护栏、视觉识别调参、坐标系变换"
tags: ["ROS2", "Gazebo", "无人机", "仿真", "iCAN"]
---
# ROS 2 两栖无人机全域巡检实战：从"飞机飞走"到全流程一次通过

> 一场从零到一的仿真项目复盘：环境踩坑、控制安全护栏、视觉识别调参、坐标系变换——每一个坑都是干货。

## 一、项目背景

比赛：**iCAN「城市应急全域巡检」**  
平台：**ROS 2 Humble + Gazebo Classic 11** 两栖无人机仿真  
任务：在模拟城市场地中完成四大任务，总分 90 分：

| 任务 | 内容 | 分值 | 完成方式 |
|------|------|------|----------|
| 任务一 | 地面随机 3 点巡检 | 10 分 | Nav2 Action `navigate_to_pose` |
| 任务二 | 穿窗 + 识别窗上目标照片 + "明显下降" | 10 分 | P 闭环飞行 + HSV 颜色检测 |
| 任务三 | 两块空中金色任务板激光照射 | 30 分 | 里程计定位 + 十字扫查 |
| 任务四 | 穿窗返回起降点降落 | 40 分 | 垂直穿窗 + 降落 |

最终成果：`demo_mode:=full_inspection` 一次串联跑通全部任务，照片稳定识别 3 秒，飞机安全落地家门口。

---

## 二、场地坐标系与关键坐标

理解场地是解题的第一步。仿真场地以米为单位，原点在家门口：

```
              北 (+y)
                │
    ┌───────────┼───────────┐
    │  起降点(0,0)           │
    │           │            │
    │   地面巡检区           │   窗户墙 x≈1.925
    │  (1.125,-0.775)等     │──────┬──────┐
    │           │            │ 窗1  │      │ 窗2
    │           │            │(1.925│      │(1.925
    └───────────┼────────────┘,-0.33)│      │,-2.37)
                │            │      │      │
              南(-y)         └──────┴──────┘
                                    │ 金色板区 (x≈3.08)
                                    │ aerial_1(3.08,-0.32)
                                    │ aerial_2(3.08,-1.36)
```

### 关键坐标表

| 目标 | 坐标 (x, y, z) | 说明 |
|------|----------------|------|
| 起降点 | (0, 0, 0.055) | 两栖车初始位置 |
| 窗 1 中心 | (1.925, -0.3275, 0.9) | 开口 z 范围 0.4~1.15 |
| 窗 2 中心 | (1.925, -2.3725, 0.9) | 开口 z 范围 0.4~1.15 |
| 窗 1 照片 | (1.815, -0.3275, 1.40) | 窗外上方，西侧面 |
| 窗 2 照片 | (2.035, -2.3725, 1.40) | 东侧面 |
| 金色板 1 | (3.080, -0.320, 0.0) | 贴地，无碰撞体 |
| 金色板 2 | (3.080, -1.355, 0.0) | 贴地，无碰撞体 |

### 相机参数

- 安装位置：机头前伸 x=0.39，z=0.13（机体系）
- FOV：1.396 rad（约 80°）
- 分辨率：800×800
- 频率：约 30Hz

### 穿窗的几何要求

窗户开口宽度约 0.4m（y 方向），高度 0.75m（z 0.4~1.15）。无人机尺寸约 0.25m，所以**穿窗必须垂直穿越**，斜切会卡死。策略：先飞到窗前对准点（x≈1.2，与窗同 y），再直线推进穿窗。

---

## 三、技术架构

### 节点分工

```
┌─────────────────┐   /uav/camera/image   ┌──────────────────────┐
│   Gazebo 仿真    │ ─────────────────────► │   vision_debug_node  │
│  (gzserver)      │                         │  HSV 颜色检测         │
│                 │ ◄───────────────────── │  发布轻量检测结果     │
└────────┬────────┘   /uav/cmd_vel          └──────────┬───────────┘
         │                                             │ /vision_debug_node/detections
         │ /odom, /uav/odom, /clock                   ▼
         │                                   ┌──────────────────────┐
         └─────────────────────────────────►│  minimal_user_node   │
                                             │  状态机+P闭环+Nav2   │
                                             └──────────────────────┘
```

### 关键设计原则

**控制节点绝不直接订阅大图像**。`/uav/camera/image` 是 800×800@30Hz，单线程执行器（`rclpy.spin_once`）订阅后会被图像处理回调淹没，饿死里程计等关键回调。正确做法：视觉节点做检测，只发布 `Float32MultiArray` 格式的 `[cx, cy, area, label, ...]` 轻量结果。

### 视觉检测消息格式

```python
# 发布格式: [cx0, cy0, area0, label0, cx1, cy1, area1, label1, ...]
# label: 0 = 黄色板子, 1 = 蓝色窗上照片
flat = []
for d in dets_plate:
    flat.extend([d.cx, d.cy, d.area, 0.0])
for d in dets_photo:
    flat.extend([d.cx, d.cy, d.area, 1.0])
```

---

## 四、核心技术点详解

### 1. 机体坐标系 vs 世界坐标系（最隐蔽的 Bug）

#### 问题现象

地面导航完成后，无人机起飞时朝错误方向狂奔，甚至撞墙。

#### 根因分析

`/uav/cmd_vel` 的 `linear.x/y` 是**机体坐标系**（x 向前，y 向左），但 P 控制器算出来的 `dx, dy` 是世界系偏差。地面 Nav2 导航后机头 yaw 可能不为 0，如果直接把世界系偏差当速度发：

```python
# 错误写法
cmd.linear.x = KP_XY * dx  # dx 是世界系，但 cmd_vel 是机体系
cmd.linear.y = KP_XY * dy
```

#### 修复：世界系 → 机体系旋转

```python
cy_, sy_ = math.cos(yaw), math.sin(yaw)
# 世界系速度向量 (vx_w, vy_w) 旋转到机体系
vx_body = vx_w * cy_ + vy_w * sy_
vy_body = -vx_w * sy_ + vy_w * cy_
```

#### yaw 计算（从四元数提取）

```python
o = self.uav_odom.pose.pose.orientation
sin_yaw = 2 * (o.w * o.z + o.x * o.y)
cos_yaw = 1 - 2 * (o.y * o.y + o.z * o.z)
yaw = math.atan2(sin_yaw, cos_yaw)
```

---

### 2. 飞行安全护栏（防止物理爆炸）

#### 事故复盘

第一次放飞时，`/uav/odom` 数值爆到 **1e20**，飞机直接飞出仿真边界，Gazebo 报 "Nan in lookupTransform"。

#### 5 条护栏

| 护栏 | 参数 | 作用 |
|------|------|------|
| 水平速度上限 | 0.35 m/s | 防止 P 增益过大导致速度爆炸 |
| 显式锁 yaw | angular.z=0 | 碰撞后 yaw 漂移导致速度控制不稳定 |
| 水平/垂直到达判定分离 | 各自阈值 | z 没到也误判到达，飞窗时撞墙 |
| 里程计看门狗 | `|x|,|y|>100` | 数值异常立即停桨中止 |
| 先垂直后水平 | 任务编排 | 每段只动一个轴，降低耦合风险 |

#### 看门狗实现

```python
def _uav_pos(self):
    if self.uav_odom is None:
        return 0.0, 0.0, 0.0
    p = self.uav_odom.pose.pose.position
    # 里程计看门狗：数值超 100m 说明仿真器状态崩溃
    if abs(p.x) > 100 or abs(p.y) > 100 or abs(p.z) > 100:
        self.get_logger().error(f"里程计异常 ({p.x:.0f},{p.y:.0f},{p.z:.0f})，中止！")
        self.cmd_vel_pub.publish(Twist())  # 紧急停桨
        raise RuntimeError("里程计异常")
    return p.x, p.y, p.z
```

---

### 3. HSV 颜色检测调参方法论

#### 调参流程

**第 1 步**：写临时脚本飞到目标点抓帧保存

```python
# 飞到照片正前方 (1.5, -0.33, 1.40)，朝东 yaw=0
self.goto(1.5, -0.3275, 1.40, target_yaw=0.0)
self.hold(...)
img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
cv2.imwrite("/tmp/window1_capture.png", img)
```

**第 2 步**：分析 HSV 分布

```python
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
photo_roi = hsv[624:800, :]  # 照片区域
for i, name in enumerate(["H", "S", "V"]):
    ch = photo_roi[:, :, i]
    print(f"{name}: min={ch.min()} max={ch.max()} mean={ch.mean():.1f}")
```

**第 3 步**：对比目标和背景的特征差异

| 区域 | H | S | V |
|------|----|----|----|
| 照片（目标） | 84~120 (mean 109) | 36~245 (mean 175) | 94~217 (mean 125) |
| 天空（背景） | ~105 | ~38 | ~209 |

**关键发现**：照片和天空的 H 很接近（都是蓝色），但 **S（饱和度）差异巨大**——照片 S≈175，天空 S≈38。用 `S≥80` 就能干净分离。

#### 最终阈值

```python
# 窗上目标照片：蓝色高饱和
# 用 S≥80 排除低饱和天空，V 下限降到 80 保留中亮度照片
BLUE_LOWER = np.array([90, 80, 80], dtype=np.uint8)
BLUE_UPPER = np.array([130, 255, 255], dtype=np.uint8)

# 金色空中任务板：金黄色
YELLOW_LOWER = np.array([15, 80, 120], dtype=np.uint8)
YELLOW_UPPER = np.array([35, 255, 255], dtype=np.uint8)
```

#### 面积过滤（防误检）

地板黄色图案约 4 万像素（误检），金色板在 1~2.5m 外约 1~5 千像素。用面积上下限一刀切：

```python
# 板子面积上下限
self.declare_parameter("vision_min_area", 500.0)
self.declare_parameter("vision_max_area", 20000.0)

# 照片面积上下限（窗前稳定约 31 万像素）
self.declare_parameter("photo_min_area", 5000.0)
self.declare_parameter("photo_max_area", 400000.0)
```

---

### 4. 视觉检测结果的类别标签

#### 最初的错误做法

用画面位置 `cy > 400` 区分板子和照片，假设板子在地面（画面下半部）、照片在窗户上方（画面上半部）。

```python
# 错误：假设照片在画面上半部
if cy > 400:
    # 板子
else:
    # 照片
```

#### 实际情况

相机安装在机体 z=0.13 处，飞机 z=1.40 时相机实际 z=1.53，比照片（z=1.40）高 0.13m。所以照片实际出现在**画面下半部**（cy≈715），全被当成板子过滤掉了。

#### 正确做法：检测结果带 label

```python
# 发布格式: [cx, cy, area, label, ...]
# label: 0 = 黄色板子, 1 = 蓝色照片
flat = []
for d in dets_plate:
    flat.extend([d.cx, d.cy, d.area, 0.0])
for d in dets_photo:
    flat.extend([d.cx, d.cy, d.area, 1.0])
```

控制端按 label 分类（步长 4）：

```python
for i in range(0, len(data) - 3, 4):
    cx, cy, area, label = data[i], data[i+1], data[i+2], data[i+3]
    if label < 0.5:
        # 黄色板子
    else:
        # 蓝色照片
```

---

### 5. P 闭环控制核心代码

```python
def uav_goto(self, target_x, target_y, target_z, target_yaw=0.0):
    """P 闭环飞到目标点 (x, y, z, yaw)。"""
    if not self.wait_for_uav_odom():
        return False

    kp_xy = float(self.get_parameter("flight_kp_xy").value)
    kp_z = float(self.get_parameter("flight_kp_z").value)
    kp_yaw = float(self.get_parameter("flight_kp_yaw").value)
    max_h = float(self.get_parameter("max_horizontal_speed").value)

    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(self, timeout_sec=0.0)
        cx, cy, cz, yaw = self._uav_pos()

        dx, dy, dz = target_x - cx, target_y - cy, target_z - cz
        horiz = math.sqrt(dx * dx + dy * dy)
        yaw_diff = self._yaw_diff(target_yaw, yaw)

        # 到达判定：水平、垂直、偏航都到位
        if horiz < thresh and abs(dz) < thresh and abs(yaw_diff) < 0.1:
            return True

        # 世界系 → 机体系
        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        vx = max(-max_h, min(kp_xy * dx * cy_ + kp_xy * dy * sy_, max_h))
        vy = max(-max_h, min(-kp_xy * dx * sy_ + kp_xy * dy * cy_, max_h))
        vz = max(-0.3, min(kp_z * dz, 0.3))
        wz = max(-0.5, min(kp_yaw * yaw_diff, 0.5))

        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = vx, vy, vz
        cmd.angular.z = wz
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.04)  # 25Hz 控制周期

    return False  # 超时
```

---

### 6. 穿窗策略：垂直穿越

#### 为什么不能斜飞

窗户开口只有 0.4m 宽，无人机尺寸 0.25m，余量仅 0.15m。斜飞会蹭到窗框，P 控制器持续推飞机撞墙。

#### 两阶段穿窗

```python
# 阶段 1：飞到窗前对准点（与窗同 y，z=0.9m）
self.uav_goto(1.2, -0.3275, 0.9)
# 阶段 2：直线推进穿窗（只动 x）
self.uav_goto(2.6, -0.3275, 0.9)  # 穿到板区一侧
```

穿窗返回时同理，先对准再垂直穿越。

---

### 7. 金色板激光照射：里程计 + 十字扫查

#### 难点

空中任务板只有 visual 没有 collision，向下激光测距无法判断是否照中。

#### 解决方案

靠里程计定位到板子正上方（z=0.9m，规则要求 ≥0.8m），然后做**十字扫查**覆盖板子面积，每个点停留 0.4 秒：

```python
x, y = point
offsets = [(0,0), (r,0), (-r,0), (0,r), (0,-r)]  # 中心+四向
for ox, oy in offsets:
    self.uav_goto(x+ox, y+oy, laser_alt, thresh=scan_thresh)
    self.hold(laser_alt, scan_dwell_sec)
    self.laser_pub.publish(Bool(data=True))  # 开激光
```

---

### 8. Nav2 地面导航集成

#### Action 调用

```python
def send_ground_nav_goal(self, waypoint):
    if not self.nav_client.wait_for_server(timeout_sec=10.0):
        return False
    x, y, yaw = waypoint
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    goal = NavigateToPose.Goal()
    goal.pose = pose
    send_future = self.nav_client.send_goal_async(goal)
    if not self.wait_for_future(send_future):
        return False
    goal_handle = send_future.result()
    if not goal_handle or not goal_handle.accepted:
        return False
    result_future = goal_handle.get_result_async()
    if not self.wait_for_future(result_future):
        return False
    result = result_future.result()
    return result.status == GoalStatus.STATUS_SUCCEEDED
```

#### 地面航点

```python
self.ground_task_waypoints = {
    "ground_0": (1.125, -1.350, 0.0),
    "ground_1": (0.000, -1.350, 0.0),
    "ground_2": (0.000, -2.700, 0.0),
    "ground_3": (1.125, -1.925, 0.0),
    "ground_4": (1.125, -0.775, 0.0),
}
```

---

## 五、环境踩坑清单

| 坑 | 根因 | 解决方案 |
|----|------|----------|
| gzclient 无窗口、相机不推流 | `GAZEBO_RESOURCE_PATH` 是**顶替语义**，把默认资源目录替换掉了，丢失 shadow_caster 材质 | 必须同时带 `/usr/share/gazebo-11` |
| TF 报 `sequence size exceeds remaining buffer` | Gazebo 和 Nav2 没一起重启，时钟跳变 | 两者必须一起重启 |
| 自动冒烟节点把飞机开走 | 启动脚本自动跑默认模式，抢占 `/uav/cmd_vel` | 启动后立即 Ctrl+C 停掉 |
| AMCL 定位漂移到 (-65, -23) | 飞行过程中 AMCL 积累误差 | 重跑前重定位到物理位置 |
| colcon build 权限失败 | 微信文件夹把文件改成只读 | `chmod -R u+w` 后再编译 |
| 飞机 yaw=90° 没正对照片 | 起飞后 yaw 漂移 | 加入 yaw 控制，target_yaw=0 |
| 照片识别不到 | HSV 阈值 V 下限太高（160），照片 V≈125 | V 下限降到 80 |
| 照片被当成板子过滤 | 用 cy 位置区分类别，照片实际在画面下半部 | 用 label 字段区分类别 |

### GAZEBO_RESOURCE_PATH 正确配置

```bash
GAZEBO_SIM_SHARE="${WORKSPACE_DIR}/binary_install/share/ican_inspection_sim"
export GAZEBO_RESOURCE_PATH="${GAZEBO_SIM_SHARE}:/usr/share/gazebo-11${GAZEBO_RESOURCE_PATH:+:${GAZEBO_RESOURCE_PATH}}"
```

---

## 六、调试技巧

### 1. 抓帧分析脚本

写一个临时 Python 脚本，订阅 `/uav/camera/image`，飞到目标点后保存图像并分析 HSV：

```python
class FlyCapture(Node):
    def __init__(self):
        super().__init__("fly_capture")
        self.bridge = CvBridge()
        self.latest_image = None
        self.create_subscription(Image, "/uav/camera/image", self.on_image, 10)
        # ... 飞到目标点 ...
        img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        cv2.imwrite("/tmp/capture.png", img)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 统计各区域 HSV
```

### 2. rqt_image_view 实时查看

```bash
rqt_image_view /uav/camera/image
# 或看视觉节点的标注输出
rqt_image_view /vision_debug_node/detection/image
```

### 3. 实时监控话题

```bash
# 查看里程计
ros2 topic echo /uav/odom --once

# 查看话题频率
ros2 topic hz /uav/odom

# 查看节点列表
ros2 node list
```

---

## 七、最终运行日志

```
[INFO] ========== 阶段 1：地面巡检 ==========
[INFO] ground_0 到达成功。
[INFO] ground_1 到达成功。
[INFO] ground_2 到达成功。
[INFO] ground_3 到达成功。
[INFO] ground_4 到达成功。
[INFO] 地面巡检结束：5/5 个航点到达。
[INFO] ========== 阶段 2：空中任务 ==========
[INFO] 到达目标 (1.10, -0.88, 0.80)
[INFO] 到达目标 (1.10, -0.88, 1.20)
[INFO] 到达目标 (1.20, -0.33, 1.20)
[INFO] 悬停识别窗上照片 ...
[INFO] 发现窗上照片：像素(156,713) 面积=44696px
[INFO] 照片稳定识别 3.0s，确认目标。
[INFO] 执行明显下降：从 0.75m 降至 0.45m ...
[INFO] 明显下降动作完成。
[INFO] 垂直穿窗 1 ...
[INFO] ========== 阶段 3：激光照射 ==========
[INFO] --- 照射 aerial_1：(3.080, -0.320) ---
[INFO] --- 照射 aerial_2：(3.080, -1.355) ---
[INFO] 激光照射结果：2/2 完成。
[INFO] 垂直穿窗 2 返回 ...
[INFO] 已落地，高度 0.08 m。
[INFO] 全流程演示结束。
```

---

## 八、实机迁移建议

仿真验证通过后，上实机需要注意：

1. **HSV 阈值微调**：实机光照与仿真不同，用抓帧脚本重新采集照片 HSV 分布，调整 BLUE 阈值
2. **P 增益调参**：实机动力响应与仿真有差异，逐步调大 `flight_kp_xy`，观察振荡
3. **速度上限**：实机可以适当提高 `max_horizontal_speed`，但务必保留看门狗
4. **照片识别稳定性**：实机可能有风扰，增加稳定识别时长到 5 秒
5. **降落检测**：实机用高度计+加速度判断落地，不要完全依赖 odom

---

## 九、经验总结

1. **坐标系永远是第一大坑**：速度、姿态、里程计的坐标系必须逐个核对，仿真能跑不代表方向对。
2. **护栏要先于功能**：没有速度上限和看门狗，一个 bug 就让飞机飞出宇宙。
3. **调参靠数据不靠猜**：写抓帧脚本分析 HSV 分布，比凭感觉调阈值快 10 倍。
4. **轻量话题解耦**：图像处理和控制分离，控制节点只订阅检测结果，性能和稳定性双赢。
5. **环境问题要治本**：遇到奇怪的 TF 报错，第一反应是重启整个环境（Gazebo + Nav2 一起），而不是调参数。
6. **穿窗必须垂直**：先对准再穿越，斜飞必卡死。
7. **类别标签优于位置先验**：不要假设目标在画面的某个位置，用检测结果自带的 label。

---

*项目仓库：ROS 2 Humble + Gazebo Classic 两栖无人机巡检仿真*  
*比赛：iCAN「城市应急全域巡检」*
```

---
