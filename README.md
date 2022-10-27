# Installation Guide

* Ubuntu 20.04 LTS Focal Fossa
* ROS Noetic Ninjemys

## Clone Packages

* Lane Planner package

```shell
roscd && cd ../src/
git clone https://gitlab.lrz.de/tas202122/tas-project/group_8.git
```

* Github packages

```shell
roscd && cd ../src/
git clone https://github.com/rst-tu-dortmund/mpc_local_planner
git clone https://github.com/MoriKen254/timed_roslaunch.git
git clone https://github.com/Etbelo/geometry2.git
```

## Install Package Dependencies

```shell
roscd && cd ..
rosdep install --from-paths src --os=ubuntu:focal --rosdistro noetic --ignore-src -r -y
```

## Build

* Activate/Deactivate `lane_planner/LaneGlobalPlanner` Debugging in [CMakeLists.txt](project_files/lane_planner/CMakeLists.txt#L5)

```C++
set(PRINT_DEBUG <ON, OFF>)
```

* Build all packages

```shell
roscd && cd ..
catkin_make
```

## Tests

```shell
roscd && cd ..
catkin_make run_tests_lane_planner
```

## Lint

Cleanup package warnings and errors

```shell
roscd && cd ..
catkin_lint . -W2 | grep "^lane_planner"
```

## Project Files

* For more information on implemented packages and project description visit [Project Files](project_files)
