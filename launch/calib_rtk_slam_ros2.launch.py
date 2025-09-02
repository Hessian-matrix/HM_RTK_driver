from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('hm_rtk')
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
                'sub_rtk_pose_topic': '/baton/rtk',
                'sub_rtk_sixdof_pose': '/baton/rtk_sixdof',
                'sub_slam_pose_topic': '/baton/stereo3/odometry',
                'pub_rtk_ex_pose_topic': '/rtk_extrinsic',
                'package_path': package_path
            }]
        )
    ])
