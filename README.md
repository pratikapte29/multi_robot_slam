# IPB ROS2 Gazebo Simulation

## Simulation

To start the simulation:

```bash
ros2 launch ipb_ros2_sim all.launch.py
```


### Optional launch arguments
Depending on your needs, consider providing the following configuration parameters when launching the simulation:
| Argument      | Default | Description |
|---------------|---------|-------------|
| `sim_config`  | `empty` | Selects the simulation scenario. Options: `indoor`, `outdoor`, `empty`, `indoor_actors`, `indoor_multi`  |
| `rviz`        | `True`  | Enables or disables RViz.|

### Robot Config
The repository provides some default config files, that you can change regarding you needs.

Modify `robot_config.yaml` file to:

  - unable/disable sensors (3D lidar, 2D lidar, front camera, upward camera, and IMU.)
  - modify sensor configuration parameters (e.g sensor noise)

### Simulation Configs

The simulation configuration is specified using the `sim_config` launch argument. Corresponding configuration files are located in the `config/simulation` directory.

| Parameter     | Description |
|--------------|-------------|
| `namespace`  | ROS2 namespace used for all nodes and topics in this simulation instance. |
| `model_type` | Robot model type (e.g., `jackal`, `husky`). |
| `init_pose` | Initial pose of the robot in the simulation (`x`, `y`, `z`, `theta`). |

---

### Robot Teleoperation
Run the teleoperation node imposing msg type TwistStamped and remapping standard cmd_vel topic to your robot’s namespace (e.g. `/P1_robot0/cmd_vel`):
```console
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -r cmd_vel:=/<namespace_from_sim_config>/cmd_vel
```

### Multi-robot simulations
Define multiple robot entries in the simulation configuration file. 
To launch a multi-robot simulation:
```console
ros2 launch ipb_ros2_sim all.launch.py sim_config:=indoor_multi max_robot_count:=2
```
| Argument      | Default | Description |
|---------------|---------|-------------|
| `max_robot_count`  | `1` | Sets the maximal number of defined robots in the simulation without needing to rebuild - enabling multi-robot scnearios. |

### Actors

Support for actors requires an additional Gazebo plugin:
https://github.com/blackcoffeerobotics/gazebo-ros-actor-plugin

```console
# install, build and source package
cd ~/ros_ws/src/ros2_masterproject/src/ 
git clone https://github.com/blackcoffeerobotics/gazebo-ros-actor-plugin.git
cd ~/ros_ws && colcon build --symlink-install
source install/setup.bash
```
In the simulation config define the following under `actors`

| Parameter     | Description |
|--------------|-------------|
| `id` | Unique identifier of the actor. |
| `model_type` | Actor model type (e.g., `female`, `male`). |
| `path_node` | Launches node if defined (e.g., `actor_path_publisher`). |
| `init_pose` | Initial pose of the actor in the simulation (`x`, `y`, `z`, `theta`). |

Actors can be launched after the simulation has been launched:
```console
ros2 launch ipb_ros2_sim actor.launch.py sim_config:=indoor_actors max_actor_count:=1 
```

| Argument      | Default | Description |
|---------------|---------|-------------|
| `max_actor_count`  | `1` | Sets the maximal number of defined actors in the simulation without needing to rebuild |



## ROS2-Related

### Workspace structure
```
ros2_masterproject/
 ├── src/
 │    ├── ipb_ros2_sim/
 │    ├── gazebo_ros_actor_plugin/
 │    ├── your_packages/
 │
 ├── build/
 ├── install/
 ├── log/
 │
 ├── docker/
 ├── README.md
 ├── .gitattributes
 ├── .gitignore
```

### Build and source packages 
```console
source /opt/ros/jazzy/setup.bash
cd ~/ros_ws
colcon build 
source install/setup.bash
```

### Limit Network communication

- To limit ROS2 communication to only the local machine and set a communication ID for ROS2:
  ```console
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export ROS_DOMAIN_ID=42
  ```

### Use Zenoh to speed up sensor updates
Change the ROS middleware to [Zenoh](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html). 

Install Zenoh for Ros-Jazzy:
```console
sudo apt install ros-jazzy-rmw-zenoh-cpp
```
Start the Zenoh node in a dedicated console:
```console
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```
Then in any other terminal run:
```console
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
before starting your ros nodes.

## Docker 
For this project implementation using the provided Docker cotainer is recommended.

```
cd ./docker
# Build container
docker compose down --remove-orphans
docker compose build

# Build (if needed) and start container
docker compose up -d

# Enter running container
docker compose exec simulation zsh

# Stop and remove container
docker compose down
```

Please notice that this has been designed to work with NVIDIA gpus, but you can modify the `compose.yaml` file and adapt it to work only with CPU or with non-NVIDIA gpus. To disable gpu usage, comment out the NVIDIA reservation block.

To let the Docker container show GUI windows on your screen, you need to allow it to connect to your host’s X server.
Run this command on your host machine:
```console
xhost +local:root
```

## GIT

Make sure Git LFS is installed and initialized:

`git lfs install` sets up Git Large File Storage (LFS) on your machine.

It:
- enables Git to handle large files (e.g. `.stl`, `.png`, `.obj`) 
- installs Git hooks so LFS works automatically
- only needs to be run once per user/machine
- defined in .gitattributes