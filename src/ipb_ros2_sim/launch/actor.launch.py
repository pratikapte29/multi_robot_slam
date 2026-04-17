import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge


# Launch Descpritopn
def generate_launch_description():
    pkg_share = get_package_share_directory("ipb_ros2_sim")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="max_actor_count",
                default_value="1",
                description="Max number of actors to spawn defined in actors_config",
            ),
            DeclareLaunchArgument(
                name="sim_config",
                default_value="empty",
                description="Simulation config defining robots and world to be spawned",
            ),
            OpaqueFunction(
                function=launch_setup_actors,
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


# Setup Actors
def launch_setup_actors(context, pkg_share, ros_gz_sim_share, *args, **kwargs):
    # FROM Launch arguments
    max_actor_count = int(LaunchConfiguration("max_actor_count").perform(context))
    sim_config = load_sim_config(context, pkg_share)

    # Config Paths
    config_paths = {
        "bridge_actor": os.path.join(
            pkg_share, "config/bridge", sim_config["bridge_actor"]
        ),
        "actor_base_path": os.path.join(pkg_share, "description", "actors"),
    }

    with open(config_paths["bridge_actor"], "r") as f:
        bridge_actor_template = f.read()

    namespace_prefix = sim_config["namespace"]

    if namespace_prefix != "":
        namespace_prefix += "_"

    namespace_prefix += "actor_"

    nodes = []
    bridge_entries = []
    all_acotr_ids = set()

    try:
        iter(sim_config["actors"])
    except TypeError:
        raise ValueError(f"Empty actor list: {sim_config['actors']}")

    # Launch actors listed in sim_config - max_actor_count is default to 1.
    for idx, actor_data in zip(range(max_actor_count), sim_config["actors"]):
        # Robot description
        actor_type = actor_data["model_type"]
        pose = actor_data["init_pose"]
        actor_id = actor_data["id"]
        if actor_id in all_acotr_ids:
            raise IndexError(f"ID {actor_id}is used multiple times")
        else:
            all_acotr_ids.add(actor_id)

        if namespace_prefix != "":
            ns = f"{namespace_prefix}{actor_id}"
            ns_slash = f"{ns}/"
        else:
            ns = ""
            ns_slash = ""

        # Bridge config
        bridge_entries.append(bridge_actor_template.replace("actor/", ns_slash))

        # Actor Template
        template_path = (
            f"{config_paths['actor_base_path']}/{actor_type}/{actor_type}.sdf"
        )
        template_basepath = f"{config_paths['actor_base_path']}/{actor_type}/"
        tmp_file_path = f"/tmp/actor_{actor_id}.sdf"

        sdf_template = load_sdf_template(template_path)
        params = {
            "actor_name": f"actor_{actor_id}",
            "actor_ns": ns_slash,
            "file_path": template_basepath,
        }
        save_sdf_template(tmp_file_path, fill_sdf(sdf_template, params))

        # Spawn Nodes
        actor_spawn = Node(
            package="ros_gz_sim",
            executable="create",
            name=f"actor_{actor_id}",
            arguments=[
                "-world",
                sim_config["world_name"],
                "-name",
                f"actor_{actor_id}",
                "-file",
                tmp_file_path,
                "-x",
                pose["x"],
                "-y",
                pose["y"],
                "-z",
                pose["z"],
                "-Y",
                pose["theta"],
            ],
            output="screen",
        )

        nodes.append(actor_spawn)

        if "path_node" in actor_data:
            if actor_data["path_node"] != "":
                actor_path_publisher_node = TimerAction(
                    period=0.5,
                    actions=[
                        Node(
                            package="ipb_ros2_sim",
                            executable=actor_data["path_node"],
                            name=f"pose_array_publisher_node_{actor_id}",
                            output="screen",
                            parameters=[{"namespace": ns_slash}],
                        )
                    ],
                )

                nodes.append(actor_path_publisher_node)

    # Create GZ-ROS2 Bridge - DO not spawn multiple clock bridges!
    actor_bridge_list = yaml.safe_load("\n".join(bridge_entries)) or []
    bridge_path = "/tmp/actor_bridge_list.yaml"
    save_yaml(bridge_path, actor_bridge_list)

    bridge_node = RosGzBridge(
        bridge_name="ros_gz_bridge_actors",
        config_file=bridge_path,
        container_name="ros_gz_container",
        create_own_container=False,
        use_composition=True,
        namespace="ros_gz_bridge",
    )

    nodes.append(bridge_node)

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


def load_sdf_template(path):
    with open(path, "r") as f:
        return f.read()


def save_sdf_template(path, data):
    with open(path, "w") as f:
        f.write(data)


def fill_sdf(template, params):
    for key, value in params.items():
        template = template.replace(f"${{{key}}}", str(value))
    return template
