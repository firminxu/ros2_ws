import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self, transformation):
        super().__init__('static_turtle_tf2_broadcaster')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self) #创建一个静态变换广播节点

        # 在启动时发布一次静态变换
        self.make_transforms(transformation)

    def make_transforms(self, transformation):
        t = TransformStamped() #创建一个转换模板

        t.header.stamp = self.get_clock().now().to_msg() #正在发布的转换时间戳
        t.header.frame_id = 'world'  #链接的父框架的名称
        t.child_frame_id = transformation[1]  #链接的子框架的名称

        t.transform.translation.x = float(transformation[2])
        t.transform.translation.y = float(transformation[3])
        t.transform.translation.z = float(transformation[4])
        quat = quaternion_from_euler(
            float(transformation[5]), float(transformation[6]), float(transformation[7]))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)


def main():
    logger = rclpy.logging.get_logger('logger')

    # 从命令行参数获取参数
    # 命令行：
    # ros2 run learning_tf2_py static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
    # 查看命令行：
    # ros2 topic echo /tf_static
    # 它使用 sys.argv 访问传递给程序的命令行参数列表。 然后它检查该列表的长度是否等于 8，
    # 这表明正在传入 7 个参数（sys.argv 的第一个元素是 python 脚本本身的名称以及路径
    # 在这里是/home/firmin/ros2_ws/install/learning_tf2_py/lib/learning_tf2_py/static_turtle_tf2_broadcaster
    if len(sys.argv) != 8:
        logger.info('Invalid number of parameters. Usage: \n'
                    '$ ros2 run learning_tf2_py static_turtle_tf2_broadcaster'
                    'child_frame_name x y z roll pitch yaw')
        #logger.info(f'O: {sys.argv[0]}') #用于查看sys.argv[0]的值
        sys.exit(1)

    if sys.argv[1] == 'world':
        logger.info('Your static turtle name cannot be "world"')
        sys.exit(2)

    # 传递参数并初始化节点, 这里是从命令行接收的参数
    rclpy.init()
    node = StaticFramePublisher(sys.argv) #
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    


