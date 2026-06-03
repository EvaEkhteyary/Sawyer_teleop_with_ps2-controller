# sawyer_teleop_with_joystick

## initial steps:

1. make a folder and clone the following:
  - robotiq (gripper) : https://github.com/ros-industrial-attic/robotiq
  - relaxed inverve kinematic: https://github.com/uwgraphics/relaxed_ik_core

2. in the same directory, clone this repository:

```
git clone https://github.com/Monkgogi/SAWYER.git 
```


## sawyer_teleop_with_joystick

### get inside the docker:
```
xhost +local:root

docker run -it --rm \
  --privileged \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/input:/dev/input \
  --device=/dev/input/js0 \
  -v /home/eva/ps2_sawyer_ws:/root/ps2_sawyer_ws \
  -v ~/ps2_sawyer_ws/src/relaxed_ik_core:/root/catkin_ws/src/relaxed_ik_core \
  -w /root/ps2_sawyer_ws \
  sawyer_noetic:latest
```

### RUN

in a terminal:
```
source devel/setup.bash
source intera.sh 
python3 src/ps2_ik_teleop/scripts/test_ps2.py
```

for an emergency press e stop and rerun this command:
```
rosrun intera_interface enable_robot.py -e
```

