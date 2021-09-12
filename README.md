# carla-ros2-bridge-docker

## Prerequisites

* EPIC user account
* Nvidia-docker

## Setup

### 1. Build the CARLA prerequisites image

```bash
docker build --build-arg EPIC_USER=<github_username> --build-arg EPIC_PASS=<github_password> -t carla-prerequisites -f Prerequisites.Dockerfile .
```

### 2. Build the final CARLA image

```bash
docker build -t carla:0.9.12 -f Carla.Dockerfile . --build-arg GIT_BRANCH=0.9.12
```

### 3. Create a CARLA package

```bash
docker run -it --rm -v ${PWD}:/workspace carla:0.9.12
# Inside the docker container
make package
cp ./Dist/CARLA_0.9.12.tar.gz /workspace
```

### 4. Build the CARLA ROS2 bridge image

```bash
docker build -t carla-foxy:0.9.12 -f Ros2Bridge.Dockerfile .
```

## Getting started

Launch the container

```bash
docker run -it --rm --privileged --gpus all --net=host --pid=host \
  -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
  carla-foxy:0.9.12
```

Terminal 1:

```bash
# /home/carla/CARLA_0.9.12
./CarkaUE4.sh
```

Terminal 2:

```bash
# /home/carla/carla-ros-bridge
source ./install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

## Trouble shooting

### Vulkan ICD

When you run `CarlaUE4.sh` on the local host, you may face an issue like blow:

```bash
4.26.2-0+++UE4+Release-4.26 522 0
Disabling core dumps.
X Error of failed request:  BadDrawable (invalid Pixmap or Window parameter)
  Major opcode of failed request:  149 ()
  Minor opcode of failed request:  4
  Resource id in failed request:  0x3c0003b
  Serial number of failed request:  300
  Current serial number in output stream:  310
terminating with uncaught exception of type std::__1::system_error: mutex lock failed: Invalid argument
Signal 6 caught.
Segmentation fault (core dumped)
```

In such case, the following may solve the issue.

```bash
export VK_ICD_FILENAMES="/usr/share/vulkan/icd.d/nvidia_icd.json"
```

### CARLA server timeout

When you run `carla_ros_bridge_with_example_ego_vehicle.launch.py`, you may face an issue like blow:

```bash
...
[bridge-1] [ERROR] [1641106937.226911738] [carla_ros_bridge]: Error: time-out of 2000ms while waiting for the simulator, make sure the simulator is ready and connected to localhost:2000
...
[carla_spawn_objects-2] [ERROR] [1641106937.815367864] [default]: Could not initialize CarlaSpawnObjects. Shutting down.
...
```

In such case, the following may solve the issue. Make sure to rebuild the source after the source is changed.

```diff
--- a/home/carla/carla-ros-bridge/src/ros-bridge/carla_ros_bridge/launch/carla_ros_bridge_with_example_ego_vehicle.launch.py
+++ b/home/carla/carla-ros-bridge/src/ros-bridge/carla_ros_bridge/launch/carla_ros_bridge_with_example_ego_vehicle.launch.py

          launch.actions.DeclareLaunchArgument(
              name='timeout',
-             default_value='2'
+             default_value='10'
          ),
```
