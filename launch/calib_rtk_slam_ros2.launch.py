from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hm_rtk',
            executable='calib_rtk_slam_node',
            name='calib_rtk_slam',
            output='screen',
            parameters=[{
                'bag_name': 'test',
                'ex_rtk_slam_x': -0.002,
                'ex_rtk_slam_y': -0.05,
                'ex_rtk_slam_z': -0.24,
                'sub_rtk_pose_topic': '/rtk_pose',
                'sub_slam_pose_topic': '/slam_pose',
                'pub_rtk_ex_pose_topic': '/rtk_extrinsic',
            }]
        )
    ])
