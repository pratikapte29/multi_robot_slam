import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


# Launch Descpritopn
def generate_launch_description():
    pkg_share = get_package_share_directory("ipb_ros2_sim")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    paths = {
        "jackal": os.path.join(
            pkg_share, "description", "jackal", "urdf", "jackal.urdf.xacro"
        ),
        "husky": os.path.join(
            pkg_share, "description", "husky", "urdf", "husky.urdf.xacro"
        ),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="jackal",
                default_value=paths["jackal"],
                description="Absolute path to jackal robot model file",
            ),
            DeclareLaunchArgument(
                name="husky",
                default_value=paths["husky"],
                description="Absolute path to husky robot model file",
            ),
            DeclareLaunchArgument(
                name="max_robot_count",
                default_value="1",
                description="Max number of robots to spawn defined in sim_config",
            ),
            DeclareLaunchArgument(
                name="sim_config",
                default_value="empty",
                description="Simulation config defining robots and world to be spawned",
            ),
            DeclareLaunchArgument(
                name="rviz",
                default_value="True",
                description="Whether to launch RViz2",
            ),
            OpaqueFunction(
                function=launch_setup_gazebo, kwargs={"pkg_share": pkg_share}
            ),
            OpaqueFunction(
                function=launch_setup_robots,
                kwargs={"pkg_share": pkg_share, "ros_gz_sim_share": ros_gz_sim_share},
            ),
        ]
    )


def load_sim_config(context, pkg_share):
    sim_config_name = str(LaunchConfiguration("sim_config").perform(context))
    sim_config_path = (
        os.path.join(pkg_share, "config/simulation", sim_config_name) + ".yaml"
    )
    sim_config = load_yaml(sim_config_path)
    return sim_config


# Setup Gazebo
def launch_setup_gazebo(context, pkg_share, *args, **kwargs):

    sim_config = load_sim_config(context, pkg_share)
    world_path = os.path.join(pkg_share, "world", sim_config["world"])
    world_name = sim_config["world_name"]

    gz_env = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=generate_gz_resource_path(pkg_share, sim_config["world"]),
    )

    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name="ros_gz_container",
        create_own_container=True,
        use_composition=True,
    )

    gz_gui = ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen")

    return [
        gz_env,
        gz_server,
        gz_gui,
        DeclareLaunchArgument("world_name", default_value=world_name),
    ]


# Setup Robots
def launch_setup_robots(context, pkg_share, ros_gz_sim_share, *args, **kwargs):
    # FROM Launch arguments
    max_robot_count = int(LaunchConfiguration("max_robot_count").perform(context))
    sim_config = load_sim_config(context, pkg_share)

    # Config Paths
    config_paths = {
        "bridge_robot": os.path.join(
            pkg_share, "config/bridge", sim_config["bridge_robot"]
        ),
        "bridge_robot_camera": os.path.join(
            pkg_share, "config/bridge", sim_config["bridge_robot_camera"]
        ),
        "bridge_sim": os.path.join(
            pkg_share, "config/bridge", sim_config["bridge_sim"]
        ),
        "rviz": os.path.join(pkg_share, "config/rviz", sim_config["rviz"]),
    }

    spawn_launch = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")

    with open(config_paths["rviz"], "r") as f:
        rviz_template = f.read()

    with open(config_paths["bridge_robot"], "r") as f:
        bridge_robot_template = f.read()

    with open(config_paths["bridge_robot_camera"], "r") as f:
        bridge_robot_camera_template = f.read()

    namespace_prefix = sim_config["namespace"]
    robotspace_prefix = sim_config["robotspace"]

    if namespace_prefix != "":
        namespace_prefix += "_"
    if robotspace_prefix != "":
        namespace_prefix += robotspace_prefix
    if namespace_prefix == "" and max_robot_count != 1:
        namespace_prefix = "ID"

    nodes = []
    bridge_entries = []
    bridge_camera_entries = []
    all_robot_ids = set()

    # Launch robots listed in sim_config - max_robot_count is default to 1.
    for idx, robot_data in zip(range(max_robot_count), sim_config["robots"]):
        # Robot description
        model_type = robot_data["model_type"]
        pose = robot_data["init_pose"]
        robot_id = robot_data["id"]
        if robot_id in all_robot_ids:
            raise IndexError(f"ID {robot_id}is used multiple times")
        else:
            all_robot_ids.add(robot_id)

        if namespace_prefix != "":
            ns = f"{namespace_prefix}{robot_id}"
            slash_ns_slash = f"/{ns}/"
            ns_slash = f"{ns}/"
        else:
            ns = ""
            slash_ns_slash = ""
            ns_slash = ""

        # RViz config
        rviz_path = f"/tmp/rviz_config_{robot_id}.yaml"
        rviz_config = yaml.safe_load(
            rviz_template.replace("robot/base_link", f"{ns_slash}base_link")
        )
        save_yaml(rviz_path, rviz_config)

        # Bridge config
        bridge_entries.append(bridge_robot_template.replace("robot/", ns_slash))
        bridge_camera_entries.append(
            bridge_robot_camera_template.replace("robot/", ns_slash)
        )

        rsp_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=f"{ns}",
            name=f"robot_state_publisher",
            parameters=[
                {
                    "robot_description": ParameterValue(
                        Command(
                            [
                                "xacro ",
                                LaunchConfiguration(model_type),
                                f" robot_namespace:={slash_ns_slash}",
                                f" robot_id:={robot_id}",
                            ]
                        ),
                        value_type=str,
                    ),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
            remappings=[("robot_description", f"{slash_ns_slash}robot_description")],
        )

        spawn_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                "world": LaunchConfiguration("world_name"),
                "topic": f"{slash_ns_slash}robot_description",
                "entity_name": f"{model_type}_{idx}",
                "x": pose["x"],
                "y": pose["y"],
                "z": pose["z"],
                "Y": pose["theta"],
            }.items(),
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            namespace=ns,
            name=f"rviz",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            arguments=["-d", rviz_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )

        nodes.extend([rsp_node, spawn_node, rviz_node])

    # Create GZ-ROS2 Bridge - DO not spawn multiple clock bridges!
    simulation_bridge_node = RosGzBridge(
        bridge_name="simulation_bridge",
        config_file=config_paths["bridge_sim"],
        container_name="ros_gz_container",
        create_own_container=False,
        use_composition=True,
        namespace="ros_gz_bridge",
    )
    nodes.append(simulation_bridge_node)

    # Create GZ-ROS2 Bridge for robot (without camera) topics
    robot_bridge_list = yaml.safe_load("\n".join(bridge_entries)) or []
    bridge_path = "/tmp/combined_robot_bridge.yaml"
    save_yaml(bridge_path, robot_bridge_list)

    robot_bridge_node = RosGzBridge(
        bridge_name="robot_bridge",
        config_file=bridge_path,
        container_name="ros_gz_container",
        create_own_container=False,
        use_composition=True,
        namespace="ros_gz_bridge",
    )
    nodes.append(robot_bridge_node)

    # Create GZ-ROS2 Bridge for robot camera topics
    robot_camera_bridge_list = yaml.safe_load("\n".join(bridge_camera_entries)) or []
    bridge_camera_path = "/tmp/combined_bridge_camera.yaml"
    save_yaml(bridge_camera_path, robot_camera_bridge_list)
    bridge_camera_node = Node(
        package="ipb_ros2_sim",
        executable="camera_bridge",
        name="camera_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_camera_path},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        namespace="ros_gz_bridge",
    )
    nodes.append(bridge_camera_node)

    return nodes


# Helpers
def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def save_yaml(path, data):
    with open(path, "w") as f:
        yaml.safe_dump(data, f)


def generate_gz_resource_path(pkg_share, world):
    base = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    added = os.pathsep.join(
        [
            os.path.join(pkg_share, os.path.pardir),
            os.path.join(pkg_share, "world", os.path.dirname(world)),
            os.path.join(pkg_share, "world", os.path.dirname(world), "models"),
        ]
    )
    return f"{base}{os.pathsep}{added}" if base else added
