from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ratslam')
    
    # Configuration file
    config_file = os.path.join(pkg_dir, 'config', 'config_stlucia.yaml')
    
    # Common parameters
    topic_root = 'stlucia'
    media_path = os.path.join(pkg_dir, 'media')
    image_file = 'irat_sm.tga'
    
    # RatSLAM nodes
    lv_node = Node(
        package='ratslam',
        executable='ratslam_lv',
        name='ratslam_lv',
        output='screen',
        parameters=[
            config_file,
            {
                'topic_root': topic_root,
                'media_path': media_path,
                'image_file': image_file,
                'use_sim_time': False
            }
        ]
    )

    pc_node = Node(
        package='ratslam',
        executable='ratslam_pc',
        name='ratslam_pc',
        output='screen',
        parameters=[
            config_file,
            {
                'topic_root': topic_root,
                'media_path': media_path,
                'image_file': image_file,
                'use_sim_time': False
            }
        ]
    )

    em_node = Node(
        package='ratslam',
        executable='ratslam_em',
        name='ratslam_em',
        output='screen',
        parameters=[
            config_file,
            {
                'topic_root': topic_root,
                'media_path': media_path,
                'image_file': image_file,
                'use_sim_time': False
            }
        ]
    )

    vo_node = Node(
        package='ratslam',
        executable='ratslam_vo',
        name='ratslam_vo',
        output='screen',
        parameters=[
            config_file,
            {
                'topic_root': topic_root,
                'use_sim_time': False
            }
        ]
    )

    return LaunchDescription([
        lv_node,
        pc_node,
        em_node,
        vo_node
    ])
