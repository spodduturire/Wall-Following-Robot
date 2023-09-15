# Wall-Following-Robot | Q-Learning | SARSA

The motivation behind this project was to design wall-following Triton LiDAR robot which can
accurrately map the environment in which the it is placed and intelligently utilize the data
received to navigate through the environment seamlessly without additional user interference.

The main approach discussed to tackle this problem is Reinforcement Learning. Specifically,
Q-Learning is implemented which is used to learn a Q-function that can be used to learn the 
overall value of taking an action in a particular situataion based on a reward system. Eventually,
Q-learning outputs a Qtable which contains values that are used to determine which actions to take
in a particular state in order to maximize overall reward in the system. We implement two separate
algorithms, the Q-Learning and the SARSA algorithms which are able to make the robot efficiently 
traverse the map following the walls.

# Environment Setup

1) We need to first clone the Stingray-Simulation repository which gives us the virtual environment
to train our robot. To do this, go to the website - https://gitlab.com/HCRLab/stingray-robotics/Stingray-Simulation
and follow all the steps as listed.

2) Next we need to create a ROS package using roscreate-pkg or catkin commands. The whole project was
run on Ubuntu 20.04.5. We need to make a 'scripts' directory in the ROS package that can hold the Python scripts
that are present in this repository.

# Running Programs

Once you add the QLearningTrain.py, QLearningTest.py, SARSATrain.py, SARSATest.py files to the scripts directory in
ROS package, follow the steps below - 

1) Open a terminal and run -> roscore

2) Open another terminal and go to the stingray package to launch gazebo, run the following commands ->
cd ~/Stingray-Simulation/
source /opt/ros/noetic/setup.bash
source stingray_setup.bash
cd catkin_ws
source devel/setup.bash
roslaunch stingray_sim wall_following_v1.launch

3) Open another terminal, run the following commands ->
cd ~/ros_package_name/
source /opt/ros/noetic/setup.bash
cd catkin_ws
source devel/setup.bash

Now you have 4 options.
A) For QLearning Training File ->
rosrun ros_package_name P2D2_QLearning_Train.launch

B) For QLearning Test File ->
roslaunch ros_package_name P2D2_QLearning_Test.launch

C) For SARSA Training File ->
roslaunch ros_package_name P2D2_SARSA_Train.launch

D) For SARSA Test File ->
roslaunch ros_package_name P2D2_SARSA_Test.launch

After running any one of these files, if you want to 
run  different launch file, open another terminal and run ->
rostopic pub /triton_lidar/vel_cmd geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
  
Also go to gazebo, go to edit tab and Reset World
Then repeat step3 from the beginning if you need to run any of the other launch files.

# YOUTUBE LINKS

QLearningTest Videos -> https://youtu.be/-6zfnMSkQIE, https://youtu.be/G2plJTXGen0
SARSATest Videos -> https://youtu.be/htHpp2eVvcQ, https://youtu.be/_coBjrYad44, https://youtu.be/V7sGDchkU20
