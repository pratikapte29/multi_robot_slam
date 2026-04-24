# IPB ROS2 Gazebo Simulation
## Setup

### Git
If you are yet unfamiliar with git, we recommend to check out the [tutorial from MIT](https://missing.csail.mit.edu/2026/version-control/) or the [official manual](https://git-scm.com/docs/user-manual). There also a variety of videos on Youtube you might watch.

#### Getting the Simulation
Create your own copy of this repository using the Fork button. This ensures that you can still update changes inside your copy if we make changes to the general upstream repository.






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

#### Updating the Simulation From Upstream
Usually, you should see "Up to date with the upstream repository." on Gitlab for your forked repository. From time to time, there might be changes on the [upstream repository](https://gitlab.igg.uni-bonn.de/students/moro-master-projects-2026-ipb/ros2_masterproject/-/tree/main?ref_type=heads), which we will always push to the main branch (you don't need to check any other branches). You will then see an "Update Fork" button appearing, if the upstream repository is ahead of yours, which allows you to pull the changes into the main branch of your forked repository. Therefore, you should **NOT** make changes to the main branch of the simulation, to make it easy to update any changes. You can do the same also from command line, if you prefer, as discussed [here](https://forum.gitlab.com/t/refreshing-a-fork/32469).


#### Making Changes to the Simulation
If you want to make any changes to the simulation, e.g., for changing and saving different configurations or similar, **you should create your own branch and create commits there.** This makes it afterwards easier to merge changes on the main branch from the [upstream repository](https://gitlab.igg.uni-bonn.de/students/moro-master-projects-2026-ipb/ros2_masterproject/-/tree/main?ref_type=heads) into your code.

```
git checkout -b <your-branch-name>
```

Once you make changes to your Code you upload them using
```console
git add .
git commit -m "a commit message describing you changes"
git push origin <your-branch-name>
```
And the others can get those changes to their local checkouts using
```console
git fetch
git checkout <your-branch-name>
git pull
```
`Git` has a lot more functionality than this and it is recommended to study some of its fundamentals (branches, merging, pull requests,...) to streamline code sharing and version control within your group. 

### Docker (Recommended)

The perhaps easiest way to install all the dependencies for this project on you computer is using docker (see [here](https://docs.docker.com/manuals/)). Docker lets you install an isolated environment on your host machine according to a recipe defined in a dockerfile. In principle you can use docker on any operating system, but you may struggle to get things to run smoothly when using Windows or specifically MacOS on an ARM based processor. When you install docker to your machine, make sure docker compose comes with the installation. Then you can follow the steps below to use your docker container

```
cd <root-of-checkout>/docker
# Build container
docker compose down --remove-orphans
docker compose build --build-arg UID=$(id -u) --build-arg GID=$(id -g)

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

#### Splitting Terminals
Once you are inside your docker container, you may find yourself wanting to split your terminal so you can run multiple things in parallel. One way to do that is by running.
```console
tmux
```
Here is a cheet sheet with the important short cuts
https://www.reddit.com/r/tmux/comments/b9llk7/classic_cheatsheet_wallpaper_for_tmux_repost/


### Installing ROS2 Locally (Alternative When not Using Docker)
While we recommend to use Docker with the provided image because it already has ROS2 pre-installed, you can also install ROS2 locally on your machine following the installation instructions [here](https://docs.ros.org/en/jazzy/Installation.html) and then run everything outside Docker.


## ROS2 Workspace Setup

Now that you have ROS2 available (either through Docker or locally), you can setup and build your ROS2 workspace. In general, the [ROS2 documentation](https://docs.ros.org/en/jazzy/Tutorials.html) is quite good and extensive and you find all information there. Here, we provide the recommended basic setup for running the simulation and your own lateron implemented code.

### Workspace Structure

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

### Creating Your own Packages
As noted in the structure above, you should put your own packages for the tasks you work on into a folder on the same level as the simulation in `ipb_ros2_sim`. **The folder** `your_packages` **should be its own Git repository, which is created by you.** The repository can then contain multiple ROS2 packages, e.g., like this

```
your_packages/
 ├── localization/
 ├── planning/
 ├── control/
 ├── ...
```
which you implement yourself to solve the task of your project. Git works well with repositories that are inside another repository, you just need to ensure that you are in the correct folder (either `ros2_masterproject` or `your_packages`), depending on to which repository you want to commit or fetch changes.

**Note:**
When making changes to the code, you can modify it outside the Docker container. We generally recommend [VSCode](https://code.visualstudio.com/) as an editor.

### Build and Source Packages 

Before running any ROS2 code you need to build and source it. 

**If you use the provided Docker container**, you first need to start and enter the running container, and then run:
```console
source /opt/ros/jazzy/setup.zsh
cd ~/ros_ws
colcon build  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.zsh
```

**If you installed ROS2 locally instead**, you need to first move to the top-level folder of the [workspace](#workspace-structure) and then run
```console
source /opt/ros/jazzy/setup.bash
cd <path/to/ros2_masterproject>
colcon build  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```
This assumes you use Bash (default on Ubuntu), such that the ending here is `.bash` instead of `.zsh` as in the instructions for the Docker container, which uses Z shell instead. More details you can find in the [ROS2 environment setup](#ros2-environment-setup).



## ROS2 Environment Setup
There are a few thing to know regarding the environment of your shell when working with ROS2.

### Bash vs Zsh
In the Docker container we use zsh instead of bash, while bash is the default on Ubuntu. When you follow tutorials from the [ROS2 documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files) ensure that you use the file ending `.zsh` instead of `.bash`, and `~/.zshrc` instead of `~/.bashrc`.

### Environment Variables
You have to set environment variables everytime again when opening a new shell. To automatically set them when creating a new shell, you can set them in the `~/.zshrc` (when using zsh in the docker container) or the `~/.bashrc` (default on Ubuntu). Note, that for the docker container you have to define variables in the file in [docker/zsh.rc](docker/zsh.rc), which defines the `~/.zshrc` inside the docker container.


### ROS Setup Files
Whenever launching any ROS applications, you need to first source the setup files:
```console
source /opt/ros/jazzy/setup.zsh   # .zsh or .bash depending on shell
```
When running any code from your ROS workspace, you need additionally to source the setup files of your ros workspace
```
source <path/to/ros_ws>/install/local_setup.zsh
```


### Limit Network Communication (RECOMMENDED) 

To limit ROS2 communication to only the local machine and set a communication ID for ROS2:
  ```console
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export ROS_DOMAIN_ID=42
  ```
This ensures that you don't accidentally communicate across machines with your peer students. See [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#the-ros-domain-id-variable) for more information.

### Use Zenoh to Speed up Sensor Updates (RECOMMENDED) 
Change the ROS middleware to [Zenoh](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html). 

Install Zenoh for Ros-Jazzy (in the docker container it is already installed):
```console
sudo apt install ros-jazzy-rmw-zenoh-cpp
```

In every terminal you run ROS-related applications in, you need to set first:
```console
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```
Start the Zenoh node in a dedicated console:
```console
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```
before starting your ros nodes.


**Note:**
Some of the steps before are already added in the [docker/zsh.rc](docker/zsh.rc) file. Feel free to modify it.


## Simulation

To start the simulation:

```bash
ros2 launch ipb_ros2_sim all.launch.py
```


### Optional Launch Arguments
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

### Multi-Robot Simulations (Only for some projects)
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




### Aruco-Based Localization
The .csv files with the global coordinates of the Aruco tags can be found here:
src/ipb_ros2_sim/world/office_world/models/aruco_marker_grid/textures/aruco_marker_corners_world.csv
src/ipb_ros2_sim/world/agri_world/models/aruco_marker_grid/textures/aruco_marker_corners_world.csv


## Issues
If you encounter any issues you cannot solve by yourself and with a bit of googling, please open an issue at: [https://gitlab.igg.uni-bonn.de/students/moro-master-projects-2026-ipb/ros2_masterproject/-/issues](https://gitlab.igg.uni-bonn.de/students/moro-master-projects-2026-ipb/ros2_masterproject/-/issues)