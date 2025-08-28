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
                'ex_rtk_slam_x': 0.1, # 外参是RTK在左目相机坐标系下的粗略坐标， XYZ-右下前
                'ex_rtk_slam_y': 0.0, # 地面小车。高程不可观，不参与优化
                'ex_rtk_slam_z': -0.5,
                'ex_rtk_slam_yaw': 30.0, # 单位：deg，从SLAM坐标系到RTK坐标系的偏航角。逆时针正，顺时针负
                'sub_rtk_pose_topic': '/rtk_pose',
                'sub_slam_pose_topic': '/slam_pose',
                'pub_rtk_ex_pose_topic': '/rtk_extrinsic',
            }]
        )
    ])
