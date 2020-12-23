# Project Course Embedded Systems

## Install

1. Install dependencies before building

```bash
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```

2. Build Catkin Package, in the main directory run:

```bash
catkin_make
```

3. Install dependencies with rosdep:

```bash
rosdep install navigation
rosdep install gazebo_sim
rosdep install robot
rosdep install teb_local_planner
```

4. Source the project: (change pathtofile)

```bash
source /pathtofile/project_course_es/devel/setup.bash
```

## Run

Launch the simulation:

```bash
roslaunch gazebo_sim sim.launch
```
