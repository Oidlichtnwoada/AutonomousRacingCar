**VU Autonomous Racing Cars (2020S) - TU Wien**

**Team 3**

Stefan Adelmann | Hannes  Brantner | Daniel Lukitsch | Thomas Pintaric

------

# Lab 8: Reinforcement Learning

The eigth lab assignment had two distinct parts:

1. We used [Google Cartographer](https://opensource.google/projects/cartographer) (an implementation of the [GraphSLAM](https://en.wikipedia.org/wiki/GraphSLAM) algorithm) to **generate a map** of an indoor environment from laser range scans. (Not on a physical racecar, but from simulated data. In our case, we re-mapped ["Levine loop"](https://github.com/f1tenth/f1tenth_labs/blob/master/f110_simulator/maps/levine.yaml), which is provided as part of the [F1tenth simulator](https://github.com/f1tenth/f1tenth_labs/tree/master/f110_simulator).)
2. We used an existing [Particle Filter implementation](https://github.com/mit-racecar/particle_filter) to localize our (simulated) vehicle inside the re-mapped "Levine loop", then drove the racecar around the track using keyboard/gamepad controls and **logged the driven trajectory** into a CSV file.  (All of this was done entirely in simulation as well.)
3. We implemented the Pure Pursuit Control Law (as described in [[Snider 2009]](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)) as a Python ROS node and created a **trajectory-following demo** (from the logged trajectory of the previous step).



## Implementation details

### Mapping (with Cartographer)

To avoid topic collisions with the F1tenth simulator, we first recorded a `.bag` file (using [Rosbag](http://wiki.ros.org/rosbag), containing the topics `/tf` and `/scan`) while driving around the simulated "Levine loop". We then ran [Cartographer](https://opensource.google/projects/cartographer) on a playback of this recording to (re-)estimate the map using GraphSLAM.

The re-mapped "Levine loop" was saved to `maps/remapped_levine_loop.yaml` and looks as follows:

![](media/adjusted_remapped_levine_loop.png)

### Localization (with a Particle Filter) & Waypoint Logging

A stand-alone demonstration of **particle filter-based self-localization** from our reconstructed map of the "Levine loop" (see image above) can be started by launching `launch/simulate_remapped_environment.launch`.

<u>DISCLAIMER:</u> It should be noted that we did not fine-tune the particle filter's parameters to our vehicle's specific motion and sensor model (see [[Thrun 2005]](https://mitpress.mit.edu/books/probabilistic-robotics), Chapter 5.4 in particular), but instead relied on the implementation's [default parameters](https://github.com/mit-racecar/particle_filter/blob/master/launch/localize.launch).

The following screenshots show our particle filter-based self-localization demo. For the initial initialization ("*kidnapped robot problem*"), the particle filter was sometimes unable to correctly localize our vehicle. In those cases, we manually provided the correct estimate via the RViz UI ("2D Pose Estimate").

![](media/pf_localization_detail.png)

The red arrows in the RViz screenshot above represent 60 randomly selected particles from the entire distribution of particles, which are published by our `particle_filter` node to the topic `\pf\viz\particles `. (This number can be changed by altering the node parameter `max_viz_particles`).

![](media/pf_localization.gif)

We drove our simulated racecar around "Levine loop" using keyboard/gamepad controls and **logged the driven trajectory** into a `.csv` file (using a slightly modified copy of the `waypoint_logger.py` node provided as part of the F1tenth repository). The entire ROS node graph of our application look as follows:

![](media/pf_rosgraph.svg)

The recorded trajectory (series of [x, y, yaw] waypoints) can be visualized with the MATLAB script `waypoint_logs/convert_raw_path.m`, which will also trim the path to remove any overlap:

![](media/original_recorded_trajectory.png)

### Pure Pursuit

The ROS node `pure_pursuit.py` is a Python implementation of the **Pure Pursuit Control Law** (as described in [[Snider 2009]](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf), in fact, we use the same variable naming scheme in our code) , which is summarized below:

![Pure Pursuit Control Law](media/pure_pursuit_control_law.svg)

The only input to the algorithm is a goal position on the tracked trajectory (in the vehicle's coordinate frame), denoted by (<img src="https://render.githubusercontent.com/render/math?math=g_{x},g_{y}">) in cartesian coordinates, or  (<img src="https://render.githubusercontent.com/render/math?math=l_{d},\alpha">) in polar coordinates. The **commanded heading angle** (as a function of time) as computed by the Pure Pursuit Control Law is denoted by<img src="https://render.githubusercontent.com/render/math?math=\delta(t)">.

We feed this heading angle into a PID-based steering controller (`drive_controller.py`, which was adapted from an earlier lab assignment) to generate `AckermannDriveStamped` cCSVommands.

![](media/pure_pursuit_detail.png)

In our demo application, the circular arc (see "Pure Pursuit Control Law" above) is drawn in the color <span style="color:cyan">**cyan**</span>, the tracked trajectory (from the exported `.csv` file) is drawn in <span style="color:lawngreen">**green**</span>, and the goal position is marked with  <span style="color:red">**X**</span><span style="color:limegreen">**Y**</span><span style="color:blue">**Z**</span> coordinate axes. 

![](media/pure_pursuit_animated.gif)

The entire ROS node graph of our application look as follows:

![](media/pure_pursuit_rosgraph.svg)

------

Our code was tested against [ROS Melodic](http://wiki.ros.org/melodic) (from the [official package repository](http://wiki.ros.org/melodic/Installation/Ubuntu)) under [Ubuntu 18.04 LTS](http://releases.ubuntu.com/18.04.4/).

## How to run the code

First, setup your catkin workspace. We assume that [ROS Melodic is already installed](http://wiki.ros.org/melodic/Installation/Ubuntu) on the system and that the workspace location is stored in the environment variable `${ROS_LAB_WORKSPACE}`.

```bash
source /opt/ros/melodic/setup.bash
mkdir -p ${ROS_LAB_WORKSPACE}/src
cd ${ROS_LAB_WORKSPACE}
catkin init --workspace ${ROS_LAB_WORKSPACE}

# Clone the new f1tenth_labs repository and remove the lab/project skeletons
git clone https://github.com/f1tenth/f1tenth_labs.git \
	${ROS_LAB_WORKSPACE}/src/f1tenth_labs
find ./src/f1tenth_labs -maxdepth 1 -type d -iname 'lab?' -or -iname 'project' | \
	xargs rm -rf	

# Clone the particle_filter repository
# Note that it might be necessary to install range_libc for the particle filter to work.
# See: https://github.com/kctess5/range_libc
git clone https://github.com/mit-racecar/particle_filter.git \
	${ROS_LAB_WORKSPACE}/src/particle_filter

# Unpack this submission into the catkin workspace
tar -xzf group3_lab6.tar.gz --directory=${ROS_LAB_WORKSPACE}/src

# Build all packages (Release)
catkin build --workspace ${ROS_LAB_WORKSPACE} -DCMAKE_BUILD_TYPE=Release
source ${ROS_LAB_WORKSPACE}/devel/setup.bash
```

### Mapping (with Cartographer)

For the installation, please follow the instructions available from:
<https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html>

<u>NOTE:</u> We created a separate (isolated) catkin workspace for our Cartographer installation. For consistency with the Google Tutorials, we use the deprecated `catkin_make` (instead of `catkin build`) inside this workspace.

##### After the installation is complete, perform the following steps:

Source the `setup.bash` file of the cartographers workspace

```bash
source ~/catkin_ws/install_isolated/setup.bash
```

Copy the folder `f110_description` to the folder `~/catkin_ws/install_isolated/share"`

This step is not needed if you intend to use the topic `/tf` directly, else you need this folder as it includes the definition of the tf-tree and the detailled robot description to recalculate the tf-tree.

```bash
cp -r f110_description ~/catkin_ws/install_isolated/share
```

Copy the two launch-files to `~/catkin_ws/install_isolated/share/cartographer_ros/launch` and adjust the paths inside of them for your system.

The launch-files are using bag-files as input for the cartographer, so you need to provide a bagfile with the suiting topics `/scan` and `/tf` (without transformations from the frame `base_link` to `map`, therefore, the broadcast of `/tf` in the simulator has to be disabled in the `params.yaml` file of the simulator).

You will also have to adjust the path to the configuration-directory inside the launch files, so it is correct for your system.

```bash
cp  *.launch ~/catkin_ws/install_isolated/share/cartographer_ros/launch
```

Copy the`f110_2d.lua` file to the folder configured in the launch-files, for example

```bash
cp f110_2d.lua ~/catkin_ws/src/cartographer_ros/cartographer_ros/configuration_files
```

Then you can record the bagfile which will be replayed as input for the cartographer

```bash
rosbag record /tf /scan
```

With the following command you can check if your bagfile is suitable for the cartographer

```bash
cartographer_rosbag_validate -bag_filename your_bag.bag
```

Then simply call roslaunch and provide the path to the bagfile like

```bash
roslaunch cartographer_ros f110_2d.launch bag_filename:=your_bag.bag
```

Now we can save the maps published by the cartographer at the topic`/map` with

```bash
rosrun map_server map_saver  -f map_name
```

And then convert it to different colors to see all the details better than in the original one

```bash
convert map_name.pgm -fuzz 34% -fill black -opaque gray converted_map_name.pgm
```

### Localization (with a Particle Filter) & Waypoint Logging

<u>Terminal 1:</u>

```bash
roslaunch group3_lab6 simulate_remapped_environment.launch
```

<u>Terminal 2:</u>

```bash
rosrun group3_lab6 waypoint_logger.py
```

### Pure Pursuit

<u>Terminal 1:</u>

```bash
roslaunch group3_lab6 pure_pursuit_simulator.launch
```

<u>Terminal 2:</u>

```bash
roslaunch group3_lab6 pure_pursuit.launch
```
