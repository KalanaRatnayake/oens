# OENS: An Octomap Based Exploration and Navigation System

This system focuses on exploration of unexplored areas. Currently the system supports single robot exploration.

This system contains 3 major components

- [Exploration Module]
- [Planning Module] 
- [Control Module] 

   [Exploration Module]: <https://github.com/KalanaRatnayake/exploration_module>
   [Planning Module]: <https://github.com/KalanaRatnayake/planning_module>
   [Control Module]: <https://github.com/KalanaRatnayake/control_module>

And uses a separate module for performance evaluation

- [Evaluator]

   [Evaluator module]: <https://github.com/KalanaRatnayake/evaluator>

for performance evaluation

## Requirements

* [Ubunut 18.04] - Prefered operating system
* [ROS melodic] - Robot Platform used for development and use
* [Git]         - As version control system

   [Ubunut 18.04]: <https://www.linuxtechi.com/ubuntu-18-04-lts-desktop-installation-guide-screenshots/>
   [ROS melodic]: <http://wiki.ros.org/melodic/Installation/Ubuntu>
   [Git]: <https://git-scm.com/>
   
<br>

## Basic Environment Setup

Follow the instructions and install Ubuntu 18.04 and ROS melodic as explained in the respective webpages. When installing Melodic, "ros-melodic-full-desktop" is prefered.

Setup Git as follows

```sh
sudo apt update
sudo apt install git
```

Setup catkin_tools as follows

```sh
sudo apt install python-catkin-tools python-catkin-pkg
```

<br>

## Dependencies

Run the following commands to setup dependencies

```sh
sudo apt install python3-pip
python3 -m pip install --upgrade setuptools grpcio-tools
```

```sh
sudo apt install ros-melodic-multimaster-fkie ros-melodic-cv-bridge ros-melodic-vision-opencv ros-melodic-rtabmap ros-melodic-rtabmap-ros libpcl-dev ros-melodic-tf2*
```

<br>

## System setup

Create a ros workspace 

```sh
mkdir -p oens/src
cd oens/src
```

and clone following repositories to 'src' folder.

```sh
git clone https://github.com/KalanaRatnayake/oens.git
git clone https://github.com/KalanaRatnayake/control_module.git
git clone https://github.com/KalanaRatnayake/exploration_module.git
git clone https://github.com/KalanaRatnayake/planning_module.git
git clone https://github.com/KalanaRatnayake/custom_msgs.git
git clone https://github.com/KalanaRatnayake/evaluator.git
git clone https://github.com/KalanaRatnayake/rviz.git
git clone https://github.com/KalanaRatnayake/simulation.git

git clone https://github.com/yujinrobot/kobuki.git
git clone https://github.com/yujinrobot/kobuki_desktop.git
git clone https://github.com/yujinrobot/kobuki_msgs.git
git clone https://github.com/yujinrobot/yujin_ocs.git
git clone https://github.com/ros-drivers/rgbd_launch.git

git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_apps.git
git clone https://github.com/turtlebot/turtlebot_simulator.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
git clone https://github.com/turtlebot/turtlebot_interactions.git
```

Run the following from the root of the workspace to setup missing dependecies.

```
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

Run the following from the root of the workspace to build the workspace

```sh
catkin build
```

<br>

## Simulation of single qbot system using qbot

First start the simulation,

```sh
source devel/setup.bash
roslaunch simulations single_qbot_large.launch
```

To start the rosbot's control systems,

```sh
source devel/setup.bash
roslaunch oens simu_qbot.launch
```

To start the RVIZ and visualizing system,

```sh
source devel/setup.bash
roslaunch rviz visualizer_single_qbot.launch
```

<br>

## Simulation of single qbot system using manual controller

First start the simulation,

```sh
source devel/setup.bash
roslaunch simulations single_qbot_large.launch
```

To start the rosbot's control systems,

```sh
source devel/setup.bash
roslaunch oens simu_qbot_manual.launch
```

To start the RVIZ and visualizing system,

```sh
source devel/setup.bash
roslaunch rviz visualizer_single_qbot.launch
```

<br>

IF you use this package, Please cite

```sh
@INPROCEEDINGS{Ratnayake2021,
  author={Ratnayake, Kalana and Sooriyaarachchi, Sulochana and Gamage, Chandana},
  booktitle={2021 5th International Conference on Robotics and Automation Sciences (ICRAS)}, 
  title={OENS: An Octomap Based Exploration and Navigation System}, 
  year={2021},
  volume={},
  number={},
  pages={230-234},
  doi={10.1109/ICRAS52289.2021.9476592}}
```