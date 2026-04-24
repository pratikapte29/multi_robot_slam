# IPB ROS2 Gazebo Simulation

## Setup

### Git

Create your own copy of this repository using the Fork button, such that you can modify it later.
Copy the forked repository to your local machine using 
```console
git clone <https-or-ssh-link-to-your-fork>
```
This repository uses the git extension `git lfs`. It enables Git to handle large files (e.g. `.stl`, `.png`, `.obj`) and is defined in the .gitattributes file.

On ubuntu you can install it using
```console
sudo apt install git-lfs
```
Initialize `git lfs` for you account (Only once per machine) using
```console
git lfs install
```
Pull all the large files in the repository by navigating to the root of your local checkout and running
```console
cd <root-of-checkout>
git lfs pull
```

Once you make changes to your Code you upload them using
```console
git add .
git commit -m "a commit message describing you changes"
git push
```
And the others can get those changes to their local checkouts using
```console
git pull
```
`Git` has a lot more functionality than this and it is recommended to study some of its fundamentals (branches, merging, pull requests,...) to streamline code sharing and version control within your group. 

### Docker

The perhaps easiest way to install all the dependencies for this project on you computer is using docker. Docker lets you install an isolated environment on your host machine according to a recipe defined in a dockerfile. In principle you can use docker on any operating system, but you may struggle to get things to run smoothly when using Windows or specifically MacOS on an ARM based processor. When you install docker to your machine, make sure docker compose comes with the installation. Then you can follow the steps below to use your docker container

```
cd <root-of-checkout>/docker
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

### Splitting Terminals
**Everything from now on should be executed inside the docker container**
Once you are inside your docker container, you may find yourself wanting to split your terminal so you can run multiple things in parallel. One way to do that is by running.
```console
tmux
```
Here is a cheet sheet with the important short cuts
https://www.reddit.com/r/tmux/comments/b9llk7/classic_cheatsheet_wallpaper_for_tmux_repost/


## ROS2-Related

This project is using ROS2, which will be automatically installed in you docker container. 

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

Before running any ROS2 code you need to build and source it.
```console
source /opt/ros/jazzy/setup.bash
cd ~/ros_ws
colcon build  --symlink-install  -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

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

### Multi-robot simulations (Only for some projects)
Define multiple robot entries in the simulation configuration file. 
To launch a multi-robot simulation:
```console
ros2 launch ipb_ros2_sim all.launch.py sim_config:=indoor_multi max_robot_count:=2
```
| Argument      | Default | Description |
|---------------|---------|-------------|
| `max_robot_count`  | `1` | Sets the maximal number of defined robots in the simulation without needing to rebuild - enabling multi-robot scnearios. |

### Actors (Only for some projects)

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



### Limit Network communication (RECOMMENDED) 

To limit ROS2 communication to only the local machine and set a communication ID for ROS2:
  ```console
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export ROS_DOMAIN_ID=42
  ```

### Use Zenoh to speed up sensor updates (RECOMMENDED) 
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

### Aruco based localisation
The .csv files with the global coordinates of the Aruco tags can be found here:
src/ipb_ros2_sim/world/office_world/models/aruco_marker_grid/textures/aruco_marker_corners_world.csv
src/ipb_ros2_sim/world/agri_world/models/aruco_marker_grid/textures/aruco_marker_corners_world.csv
