# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')
        self.logger = rclpy.logging.get_logger('logger')
        # self.declare_parameter 声明存在一个名为“target_frame”的参数,该参数的默认值为“turtle1”
        # .get_parameter_value() 获取该参数的实际值
        # .string_value 将其转换为字符串，然后将其分配给 self.target_frame
        # ROS 参数允许节点在运行时配置自身。 此代码声明并检索指定转换目标框架的参数值。 
        # 稍后在查找到该目标帧的变换或从该目标帧查找变换时可以使用此功能。
        self.target_frame = self.declare_parameter(
            'target_frame', 'turtle1').get_parameter_value().string_value

        # 创建一个 Buffer 对象并将其分配给 self.tf_buffer, 缓冲区将缓存来自转换侦听器的转换
        # 这允许节点有效地查找帧之间的变换
        # 通过初始化缓冲区，您的节点现在可以访问帧之间的坐标转换。 
        # 它可以在需要时使用 TransformListener 和缓冲区来查找转换。
        self.tf_buffer = Buffer()
        
        # 创建一个名为 self.tf_listener 的 TransformListener 对象传入 tf_buffer （缓存转换）
        # 还传入 self，它是当前节点/类实例。TransformListener 允许节点查找坐标系之间的变换
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建一个客户端以生成一只海龟
        # 创建一个名为“spawner”的 ROS 客户端
        # 客户端配置为与名为“spawn”的服务进行通信
        # 然后可以使用此客户端异步调用此“生成”服务
        # 常见用法是调用生成服务来添加新实体
        self.spawner = self.create_client(Spawn, 'spawn')
        # 存储信息海龟生成服务是否可用的布尔值
        self.turtle_spawning_service_ready = False
        # 如果海龟成功生成
        self.turtle_spawned = False

        # 创建turtle2速度发布者
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # 每秒调用on_timer函数
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # 将帧名称存储在将用于计算变换的变量中        
        from_frame_rel = self.target_frame
        to_frame_rel = 'turtle2'

        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                # 查找target_frame和turtle2帧之间的转换, 并发送速度命令让turtle2到达target_frame
                try:
                    # 如果turtle2帧在5秒内没有转换到target_frame, 则抛出TransformException异常
                    # 如果turtle2帧在5秒内转换到target_frame, 则返回一个Transform对象, 包含了target_frame和turtle2帧之间的变换信息
                    when = self.get_clock().now() - rclpy.time.Duration(seconds=5.0)
                    # lookup_transform_full方法返回一个Transform对象, 包含了target_frame和turtle2帧之间的变换信息
                    t = self.tf_buffer.lookup_transform_full(
                            target_frame=to_frame_rel,
                            target_time=rclpy.time.Time(),
                            source_frame=from_frame_rel,
                            source_time=when,
                            fixed_frame='world',
                            timeout=rclpy.duration.Duration(seconds=0.05))

                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return
                except (LookupException, ConnectivityException, ExtrapolationException):
                    self.get_logger().info('transform not ready')
                    raise
                    return
                
                # 发送速度命令让turtle2到达target_frame
                msg = Twist()
                scale_rotation_rate = 1.0
                
                # 角速度计算公式: w = atan2(y, x) * scale_rotation_rate,
                # 与使用 atan(y/x) 相比，它更准确并避免了边缘情况。 例如，atan2 可以避免 x 为 0 时出现的问题。
                msg.angular.z = scale_rotation_rate * math.atan2(
                    t.transform.translation.y,
                    t.transform.translation.x)
                
                # 速度计算公式: v = sqrt(x^2 + y^2) * scale_forward_speed,
                scale_forward_speed = 0.5
                msg.linear.x = scale_forward_speed * math.sqrt(
                    t.transform.translation.x ** 2 +
                    t.transform.translation.y ** 2)

                self.publisher.publish(msg)
            else:
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().info('Spawn is not finished')
        else:
            if self.spawner.service_is_ready():
                # 使用海龟名称和坐标初始化请求，请注意，x、y 和 theta 在turtlesim/srv/Spawn 中被定义为浮点数
                # “Spawn.Request()”调用正在构建一个空的请求消息，然后可以使用参数填充该消息，例如要生成的实体
                # 类型、其初始姿势/位置、名称等。然后，该请求消息通常会传递到 ROS 服务实际生成新实体。
                request = Spawn.Request()
                request.name = 'turtle2'
                request.x = 4.0
                request.y = 2.0
                request.theta = 0.0
                # 调用请求
                # 之前创建了 Spawn.Request() 对象来定义要生成的内容的详细信息
                # self.spawner.call_async() 调用 self.spawner 定义的异步生成服务
                # 将请求对象传递给 call_async() 将生成详细信息发送到服务
                # 调用后，self.turtle_spawning_service_ready 设置为 True 以指示服务已准备就绪
                # 这似乎是使用 ROS 服务将海龟模拟实体异步生成到请求定义的环境中。 异步调用服务在 ROS 
                # 中很常见，以避免阻塞/等待长时间操作（例如生成）完成。
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                # 检查服务是否准备好
                self.get_logger().info('Service is not ready')


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
