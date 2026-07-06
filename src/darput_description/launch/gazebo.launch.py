from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # 1. Dapatkan direktori package darput_description
    share_dir = get_package_share_directory('darput_description')
    rviz_config_file = os.path.join(share_dir, 'config', 'default.rviz')

    # 2. Proses file Xacro menjadi URDF
    xacro_file = os.path.join(share_dir, 'urdf', 'darput.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # 3. Argumen untuk menyalakan Gazebo GUI atau Headless
    gui_arg = DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI")
    gui = LaunchConfiguration("gui")

    # 4. Include launch file Gazebo Fortress (Dengan GUI)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={"gz_args": ["-r -v 3 empty.sdf"]}.items(),
        condition=IfCondition(gui),
    )

    # 5. Include launch file Gazebo Fortress (Tanpa GUI / Headless)
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={"gz_args": ["--headless-rendering -s -r -v 3 empty.sdf"]}.items(),
        condition=UnlessCondition(gui),
    )

    # 6. Bridge untuk sinkronisasi waktu (/clock) dari Gazebo ke ROS 2
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # 7. Node untuk men-spawn URDF robot 'darput' ke dalam Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "darput",
            "-allow_renaming", "true",
            "-z", "0.5",  # Di-spawn 0.5 meter di atas tanah agar tidak bentrok dengan lantai
        ],
    )

    # 8. Robot State Publisher (Penting: set use_sim_time True)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_urdf}],
    )
    

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )

    imu_reader = Node(
        package='darput_description',
        executable='imu_reader',
        name='imu_reader',
        output='screen'
    )

    UI_Interface = Node(
        package='darput_description',
        executable='UI_Interface',
        name='UI_Interface',
        output='screen'
    )

    foot_contact_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/contact_right@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            "/contact_left@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
        ],
        output="screen",
    )

    foot_contact_publisher = Node(
        package='darput_description',
        executable='foot_sensor',
        name='foot_sensor',
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        robot_state_publisher_node,
        gz_spawn_entity,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        rviz_node,
        imu_broadcaster_spawner,
        imu_reader,
        UI_Interface,
        foot_contact_bridge,
        foot_contact_publisher,
    ])