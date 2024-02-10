# Nuturtle Control
turtle_control, odometry, and circle nodes for simulating and moving the real turtlebot in a circle.

## Quickstart
The following steps were taken to connect to and run the nodes on the real turtlebot:

### On Computer:
1. `cd` into ROS workspace
2. cross compile and send code to the robot by: 
```
./aarch64 colcon aarch64

rsync -av --delete aarch64_install/ msr@raphael:/home/msr/install
```
3. When connected to turtlebot and ready to run:
```
ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=circle robot:=none use_rviz:=true
```
4. Start control service
```
ros2 service call /control nuturtle_control/srv/Control "{velocity: 0.15, radius: 1.0}"
```
5. To stop turtlebot
```
ros2 service call /stop std_srvs/srv/Empty
```
### On Turtlebot:
1. Connect to the turtlebot:
```
ssh -oSendEnv=ROS_DOMAIN_ID msr@raphael
```
2. Launch
```
source install/setup.bash
ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=none robot:=localhost use_rviz:=false
```

## Nodes
- turtle_control: converts robot sensor data into joint states and twists into wheel commands that are passed to nusim and odometry

- odometry: calculates the odometry from the published joint states

- circle: controls the robot to move in a circle and provides the control, reverse, and stop services

## Simulation and Launchfile
The start_robot.launch.xml takes in conditional arguments to start the robot, simulation, and controls. These are the arguments available:

- robot: 
    - 
    - nusim: launches the nusim simulator with both red and blue robots
    - localhost: launches the odometry and turtle_control nodes for the turtlebot3
    - none: launches no nodes

- cmd_src:
    -
    - teleop: launches turtlebot3_teleop node
    - circle: launches the circle node
    - none: launches no additional nodes

- use_rviz:
    -
    - false: don't launch rviz
    - true: launch rviz with specifc configs depending on robot variable


## Demo Video
The following demo video was run on turtlebot3 Raphael


https://github.com/ME495-Navigation/slam-project-ishani-narwankar/assets/42013894/3ddd2929-6ae6-47f0-885e-88e0e4ae6e96


## Odometry Results
As shown by the turtlebot running into the cup at the end of the video, it is clear that the turtlebot3 experiences some drift. The following results showcase the odom error from the beginning of the video to the end.

Starting odom values:
```
x: 0.242
y: -0.02
z: 0.0
```

Ending odom values:
```
x: 0.335
y: 0.078
z: 0.0
```
Error:
```
x: 0.093
y: 0.098
z: 0.0
```
