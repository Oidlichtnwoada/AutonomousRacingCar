**VU Autonomous Racing Cars (2020S) - TU Wien**

**Group 3**

Stefan  Adelmann (01633044)
Hannes  Brantner (e01614466)
Daniel  Lukitsch (01634053)
Thomas Pintaric (09620608)

# Lab 5: Scan Matching

### ROS Package & Nodes

The submitted ROS package is named **group3_lab5** and contains the node `scan_matcher`.

<!-- TODO: Short description... -->

------

Our code was tested with [ROS Melodic](http://wiki.ros.org/melodic) under [Ubuntu 18.04 LTS](http://releases.ubuntu.com/18.04.4/).

### How to run the code

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

# Unpack this submission into the catkin workspace
tar -xzf group5_lab5.tar.gz --directory=${ROS_LAB_WORKSPACE}/src

# Build all packages (Release)
catkin build --workspace ${ROS_LAB_WORKSPACE} -DCMAKE_BUILD_TYPE=Release
source ${ROS_LAB_WORKSPACE}/devel/setup.bash
```

Launch the `f110_simulator` from [f1tenth](https://github.com/f1tenth) / **[f1tenth_labs](https://github.com/f1tenth/f1tenth_labs)** using the included launch script [launch/simulator.launch](launch/simulator.launch).

```bash
roslaunch group3_lab5 simulator.launch
```

In a separate console, start the `scan_matcher` node using the included launch script [launch/scan_matcher.launch](launch/scan_matcher.launch).

```bash
roslaunch group3_lab5 scan_matcher.launch
```

Expected result:

<!-- TODO: Insert screenshot here... -->