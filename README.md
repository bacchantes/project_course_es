# Project Course Embedded Systems

## Install

Install dependencies before building

```bash
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```

Catkin Package, in the main directory run:

```bash
catkin_make
```

Install dependencies with rosdep:

```bash
rosdep install navigation
rosdep install gazebo_sim
rosdep install robot
rosdep install teb_local_planner
```

## Run

Source the project: (change pathtofile)

```bash
source /pathtofile/project_course_es/devel/setup.bash
```
