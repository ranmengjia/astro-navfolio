---
title: 'ROS2 服务通信实战：从零跑通一个人脸检测服务的完整流程'
date: 2026-10-10
description: '服务通信一条流水线的人脸检测实战'
tags: ["ROS2", "服务通信", "参数"]
---
# ROS2 服务通信实战：从零跑通一个人脸检测服务的完整流程

> 学习 ROS2 通信机制时，网上教程大多是"话题一篇、服务一篇、参数一篇"分开讲的，但真正动手时你会发现：**它们其实是同一条流水线上的不同工位**。这篇博客以"人脸检测服务"为实战项目，把功能包、接口定义、服务端、客户端、参数、命令行调试串成一条完整的创建流程，照着走一遍就能跑通。

## 全景图：一条流水线

在写第一行代码之前，先看整体流程。ROS2 里任何一个通信功能（话题/服务/参数），创建过程都是这条链路：

```
创建工作空间
   │
   ▼
创建接口包，定义 .srv / .msg 接口 ──► 编译接口包 ──► ros2 interface show 验证
   │
   ▼
创建功能包（写节点的地方），声明依赖
   │
   ▼
写服务端节点 ──► 写客户端节点 ──► setup.py 注册入口点
   │
   ▼
colcon build ──► source ──► ros2 run 测试 ──► ros2 service/param 命令行调试
   │
   ▼
需要可配置项？加参数（声明 → 读取 → 命令行/代码动态修改）
```

下面按这条链路一步步走。

## 第一步：创建工作空间与接口包

接口（.srv/.msg）和节点代码**分属两个包**，这是 ROS2 的工程惯例：接口包独立编译，可以被任意多个功能包复用。

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
# 接口包必须是 ament_cmake 类型
ros2 pkg create --build-type ament_cmake chapt4_interfaces
# 功能包用 ament_python
ros2 pkg create --build-type ament_python chapt4_face --dependencies rclpy chapt4_interfaces sensor_msgs
```

`--dependencies` 声明的依赖会自动写进 `package.xml`，将来 `colcon build` 和代码 import 都靠它。

## 第二步：定义服务接口并编译

在 `chapt4_interfaces` 下新建 `srv/FaceDetector.srv`：

```yaml
sensor_msgs/Image image
---
int8 number
float32 use_time
int32[] top
int32[] right
int32[] bottom
int32[] left
```

三条杠 `---` 上方是**请求**，下方是**响应**。这一份文件同时描述了"客户端发什么、服务端回什么"。

然后改 `CMakeLists.txt`，把接口注册给 rosidl（ROS2 的接口编译器）：

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FaceDetector.srv"
  DEPENDENCIES sensor_msgs
)
```

编译并验证：

```bash
cd ~/ros2_ws
colcon build --packages-select chapt4_interfaces
source install/setup.bash
ros2 interface show chapt4_interfaces/srv/FaceDetector
```

能看到字段原样打印，说明接口包编译成功，`chapt4_interfaces.srv.FaceDetector` 这个 Python 类已经生成，接下来两个节点都 import 它。

## 第三步：写服务端节点（提供能力的一方）

服务端四件事：**创建节点 → 创建服务 → 写回调 → spin 等活**。

```python
import time
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from chapt4_interfaces.srv import FaceDetector
from rcl_interfaces.msg import ParameterDescriptor


class FaceDetectorServer(Node):
    def __init__(self):
        super().__init__('face_detector_server')
        # ① 创建服务：接口类型 + 服务名 + 回调
        self.srv = self.create_service(
            FaceDetector, 'face_detector', self.face_detector_callback)
        # ② 一次性初始化的重资源放构造函数：格式转换器 + 人脸检测器
        self.bridge = CvBridge()
        self.detector = cv2.CascadeClassifier(
            '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml')
        # ③ 声明参数（可配置项）
        self.declare_parameter('scale_factor', 1.1,
            ParameterDescriptor(description='Haar检测的scaleFactor'))
        self.declare_parameter('min_neighbors', 5,
            ParameterDescriptor(description='Haar检测的minNeighbors'))

    def face_detector_callback(self, request, response):
        start = time.time()
        # ④ ROS Image → OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(request.image, 'bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # ⑤ 检测，参数从 param 系统读取而非写死
        scale = self.get_parameter('scale_factor').value
        neigh = self.get_parameter('min_neighbors').value
        faces = self.detector.detectMultiScale(gray, scaleFactor=scale,
                                               minNeighbors=int(neigh))
        # ⑥ [x,y,w,h] 转成接口定义的四个数组
        response.number = len(faces)
        response.left  = [x for x, y, w, h in faces]
        response.top   = [y for x, y, w, h in faces]
        response.right = [x + w for x, y, w, h in faces]
        response.bottom= [y + h for x, y, w, h in faces]
        response.use_time = time.time() - start
        return response          # 必须 return，否则客户端收不到


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorServer()
    rclpy.spin(node)             # 挂起等待请求
    rclpy.shutdown()
```

## 第四步：写客户端节点（使用能力的一方）

客户端四件事：**创建客户端 → 等服务上线 → 发请求 → 等结果**。

```python
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from chapt4_interfaces.srv import FaceDetector


class FaceDetectorClient(Node):
    def __init__(self):
        super().__init__('face_detector_client')
        # ① 服务名必须和服务端完全一致：'face_detector'
        self.cli = self.create_client(FaceDetector, 'face_detector')
        # ② 等服务端上线，防止请求打空
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务未启动，继续等待...')
        self.req = FaceDetector.Request()
        self.bridge = CvBridge()

    def send_request(self):
        # ③ 本地图片 → ROS Image 消息（服务端 imgmsg_to_cv2 的反向操作）
        cv_image = cv2.imread('/home/user/chapt4/1.jpg')
        self.req.image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        # ④ 异步发送 + 阻塞等结果，这个套路固定
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorClient()
    response = node.send_request()
    node.get_logger().info(f'检测到 {response.number} 张人脸，耗时 {response.use_time:.4f}s')
    node.destroy_node()
    rclpy.shutdown()
```

## 服务端 vs 客户端：创建流程对照表

两个节点写完，回头对照官方总结的六步流程，会发现它们像镜子一样步步对应——前两步完全相同，中间两步角色互换，最后一步殊途同归：

| 服务端 | 客户端 |
|:---:|:---:|
| 1. 编程接口初始化 | 1. 编程接口初始化 |
| 2. 创建节点并初始化 | 2. 创建节点并初始化 |
| 3. 创建服务器端对象 | 3. 创建客户端对象 |
| 4. 通过回调函数处理服务 | 4. 创建并发送请求数据 |
| 5. 向客户端反馈应答结果 | 5. 等待服务器端应答数据 |
| 6. 销毁节点并关闭接口 | 6. 销毁节点并关闭接口 |

对照代码看：第 1、2 步对应 `rclpy.init` + `super().__init__()`（两边一样）；第 3 步对应 `create_service` vs `create_client`（镜像）；第 4、5 步对应"回调里处理并 return response" vs "call_async 后 spin_until_future_complete 等结果"（一个供给、一个索取）；第 6 步对应 `rclpy.shutdown`（收尾一致）。**记住这张表，以后写任何服务（不只是人脸检测）都是这六步换个回调逻辑而已。**

## 第五步：注册入口点 + 编译运行

Python 节点要在 `setup.py` 的 `entry_points` 登记，`ros2 run` 找不到入口就跑不起来：

```python
entry_points={
    'console_scripts': [
        'face_detector_server = chapt4_face.face_detector_server:main',
        'face_detector_client = chapt4_face.face_detector_client:main',
    ],
},
```

格式是 `'可执行名 = 包.模块:函数'`。然后编译、source、两个终端分别跑：

```bash
colcon build --packages-select chapt4_face
source install/setup.bash
# 终端1
ros2 run chapt4_face face_detector_server
# 终端2
ros2 run chapt4_face face_detector_client
```

## 第六步：命令行调试——不写客户端也能测服务端

服务跑起来后，`ros2 service` 全家桶可以直接在终端验证，**不用写一行客户端代码**：

```bash
ros2 service list                      # 看到 /face_detector
ros2 service type /face_detector       # chapt4_interfaces/srv/FaceDetector
ros2 service find chapt4_interfaces/srv/FaceDetector   # 反查哪些服务用这个接口

# 直接调用！YAML 格式填请求参数
ros2 service call /face_detector chapt4_interfaces/srv/FaceDetector \
  "{image: {width: 640, height: 480, encoding: 'rgb8', step: 1920, data: [0]}}"
```

调试阶段这个技巧非常省时间：先保证服务端正确，再写客户端。

## 第七步：加参数——让写死的数值活起来

上面代码里 `scaleFactor=1.1` 写死在回调里，想调整就得改代码重新编译。参数系统解决的就是这个问题，而且它**底层就是一个服务调用**：每个声明了参数的节点会自动暴露 `/节点名/set_parameters` 服务。

**使用链路同样是四步：**

```bash
# 1. 声明：代码里 declare_parameter('scale_factor', 1.1)
# 2. 读取：回调里 self.get_parameter('scale_factor').value
# 3. 查看：命令行
ros2 param list /face_detector_server
ros2 param get /face_detector_server scale_factor
# 4. 修改：启动时覆盖 or 运行时动态改
ros2 run chapt4_face face_detector_server --ros-args -p scale_factor:=1.05
ros2 param set /face_detector_server min_neighbors 3
```

运行时改的值不放心？加一个**校验回调**，非法值直接拦截（例如 `scale_factor <= 1.0` 会让 OpenCV 崩溃）：

```python
from rcl_interfaces.msg import SetParametersResult

self.add_on_set_parameters_callback(self.parameter_callback)

def parameter_callback(self, params):
    for param in params:
        if param.name == 'scale_factor' and param.value <= 1.0:
            return SetParametersResult(successful=False, reason='必须大于1.0')
    return SetParametersResult(successful=True)
```

更进一步，**用代码改别的节点的参数**，官方封装好的 `AsyncParametersClient` 三行搞定：

```python
from rclpy.parameter_client import AsyncParametersClient
from rclpy.parameter import Parameter

self.param_client = AsyncParametersClient(self, '/face_detector_server')
future = self.param_client.set_parameters([
    Parameter('scale_factor', Parameter.Type.DOUBLE, 1.05)])
```

## 流程串联总结

把整条链路浓缩成一张速查表：

| 阶段 | 动作 | 关键命令/代码 |
|------|------|---------------|
| 1 建包 | 接口包(cmake) + 功能包(python) | `ros2 pkg create` |
| 2 定接口 | 写 .srv，编译，验证 | `ros2 interface show` |
| 3 服务端 | `create_service` + 回调 + `return response` | `rclpy.spin` |
| 4 客户端 | `create_client` + `wait_for_service` + `call_async` | `spin_until_future_complete` |
| 5 注册运行 | setup.py 入口点 → 编译 → 两个终端跑 | `colcon build` + `ros2 run` |
| 6 调试 | 命令行直接调服务 | `ros2 service list/type/call` |
| 7 参数化 | 声明→读取→命令行改→校验→代码改 | `ros2 param get/set` |

**两种通信方式的选型**也藏在流程里：如果你不需要"有去有回"，只是周期性发数据（传感器流、控制指令），把第三、四步换成 `create_publisher` / `create_subscription`，接口包里写 `.msg` 而不是 `.srv`，其余流程一模一样——这就是话题通信。需要长任务+进度反馈（导航、机械臂），则升级成动作（Action），它是"3 个服务 + 2 个话题"的组合。

掌握这条流水线后，遇到任何新的通信需求，你都是在重复同一个动作：**定义接口 → 实现两端 → 编译 → 命令行验证 → 参数化**。
