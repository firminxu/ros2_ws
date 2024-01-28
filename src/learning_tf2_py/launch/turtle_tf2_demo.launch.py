from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        #这个节点生成第一只乌龟
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        
        #启动broadcaster1,这个节点会订阅/turtle1/cmd_vel话题，并将其发送到/turtle1/pose话题上
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster', #可执行脚本名称，在setup文件中定义
            name='broadcaster1',                #生成的节点名称
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        
        # 启动broadcaster2,这个节点会订阅/turtle2/cmd_vel话题，并将其发送到/turtle2/pose话题上
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        # 这个节点生成第二只乌龟名字，自动命名为turtle2
        # 生成turtle2时会自动生成/turtle2/cmd_vel /turtle2/color_sensor /turtle2/pose三个话题
        # 它定义了一个 Node 类型，具有包、可执行文件、名称等属性
        # 参数列表将“target_frame”参数传递给节点
        # 该参数来自名为“target_frame”的 LaunchConfiguration      
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),
    ])