from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hm_rtk',
            executable='rtk_driver_node',
            name='HM_RTK',
            output='screen',
            parameters=[{
                # NTRIP和RTK通信参数
                'ntrip_ip': '203.107.45.154',
                'ntrip_port': 8002,
                'ntrip_user': 'qxshu0032164',
                'ntrip_passwd': '1679484',
                'ntrip_mountpoint': 'AUTO',
                'rtk_port': '/dev/ttyS0',
                'rtk_baudrate': 460800,
                'pub_rtk_nmea_topic': '/rtk_nmea',
                'pub_rtk_ex_pose_topic': '/rtk_extrinsic',
                
                # 6DOF外参配置：T_camL <- RTK ，camL坐标系XYZ-右下前， RTK坐标系XYZ-前左上
                # 平移参数 (单位: 米)
                'ex_rtk_slam_tx': -0.002,
                'ex_rtk_slam_ty': -0.05,
                'ex_rtk_slam_tz': -0.24,
                
                # 旋转参数 (四元数表示: x, y, z, w)
                'ex_rtk_slam_qx': 0.0,
                'ex_rtk_slam_qy': 0.0,
                'ex_rtk_slam_qz': 0.0,
                'ex_rtk_slam_qw': 1.0,
            }]
        )
    ])
