import os
from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    use_stereo = LaunchConfigurationEquals('use_stereo', 'true')

    settings_base_path = os.path.join(os.getcwd(), 'src', 'vslam', 'settings')
    vslam_system = Node(
        package='vslam',
        executable='system',
        parameters=[
            {'settings_path': os.path.join(settings_base_path, 'stereo_settings.yaml'), 'mono_video_topic': 'null'}
            if use_stereo else
            {'settings_path': os.path.join(settings_base_path, 'mono_settings.yaml'), 'mono_video_topic': '/left_camera/image_raw'}
        ],
    )

    stereo_combiner = Node(
        package='vslam',
        executable='stereo_combiner',
    )

    map_visualization_splitter = Node(
        package='vslam',
        executable='map_visualization_splitter',
    )
    
    grid_mapper = Node(
        package='vslam',
        executable='grid_mapper',
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_stereo', default_value='true', description='Use stereo camera. If false, will use monocular camera.'))
    ld.add_action(vslam_system)
    if use_stereo:
        ld.add_action(stereo_combiner)
    ld.add_action(map_visualization_splitter)
    ld.add_action(grid_mapper)

    return ld

