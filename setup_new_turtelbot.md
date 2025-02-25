

# Creat a workspace

```bash
cd ~
mkdir ros2_ws
cd ros2_ws
mkdir src
colcon build
``` 

# set it as source 
```bash
source ~/ros2_ws/install/setup.bash
```
isntall pakages
```bash
sudo apt update
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3
```
clone pakages
```bash
cd ~/ros2_ws/src
git clone https://github.com/DTU-IPL/Course-34761-robot-autonomy.git
```

```bash
source ~/ros2_ws/install/setup.bash
```

buidl

```bash
cd ~/ros2_ws
colcon build
```

```bash
source ~/ros2_ws/install/setup.bash
```

exort 
```bash
export ROS_DOMAIN_ID=11
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix my_turtlebot)/share/my_turtlebot/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
``` 

lunche sim
```bash
ros2 launch my_turtlebot turtlebot_simulation.launch.py
```


