from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
        # 创建节点
    hm_rtk_node = Node(
        package='hm_rtk',
        executable='rtk_driver_node',
        # name='HM_RTK',
        output='screen',
        parameters=[
            {
                'ntrip_ip': "203.107.45.154",
                'ntrip_port': 8002,
                'ntrip_user': "qxvwbt0063725",
                'ntrip_passwd': "xxxx",
                'ntrip_mountpoint': "AUTO",
                'rtk_port': "/dev/ttyUSB0",
                'rtk_baudrate': 460800,
                'pub_rtk_nmea_topic': "/rtk_nmea",
                'pub_rtk_ex_pose_topic': "/rtk_extrinsic",
                'ex_rtk_slam_x': -0.002,
                'ex_rtk_slam_y': -0.05,
                'ex_rtk_slam_z': -0.24
            }
        ]
    )

    return LaunchDescription([
        hm_rtk_node
    ])