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
