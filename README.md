## Joint Perception and Planning For Efficient Obstacle Avoidance Using Stereo Vision

This repository contains a C++ implementation of JPP for local obstacle avoidance using stereo cameras. A ROS wrapper is also included in the `ROS/` folder.

- Author: Sourish Ghosh (sourishg AT iitkgp DOT ac DOT in)

## 1. Dependencies

- A C++ compiler (*e.g.*, [GCC](http://gcc.gnu.org/))
- [cmake](http://www.cmake.org/cmake/resources/software.html)
- [popt](http://freecode.com/projects/popt)
- [libconfig](http://www.hyperrealm.com/libconfig/libconfig.html)
- [Boost](http://www.boost.org/)
- [OpenCV](https://github.com/opencv/opencv)
- [OpenMP](http://www.openmp.org/)

Use the following command to install dependencies:

```bash
sudo apt-get install g++ cmake libpopt-dev libconfig-dev libboost-all-dev libopencv-dev python-opencv gcc-multilib
```

For compiling and running the ROS wrapper, install [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu).

## 2. Compiling

Clone the repository:

```bash
https://github.com/umass-amrl/jpp
```

The script `build.sh` compiles the JPP library:

```bash
cd jpp
chmod +x build.sh
./build.sh
```

This will create `libjpp.so` inside the `lib/` folder and put a binary file `jpp` inside the `bin/` folder. For compiling the ROS wrapper, `rosbuild` is used. 
Add the path of the ROS wrapper to `ROS_PACKAGE_PATH`. Replace `PATH` by the actual path where you have cloned the repository:

```bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/PATH/jpp/ROS
```

Execute the `build_ros.sh` script:

```bash
chmod +x build_ros.sh
./build_ros.sh
```
