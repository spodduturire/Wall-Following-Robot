#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelState, ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_srvs.srv import Empty
import tf
import math
import random

q_table = []
zones = []

#state-action combinations for a single state
for i in range(28):
    s = [0] * 3
    q_table.append(s)

def reset_robot(linear_x, linear_y, orientation_z):
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Create a ModelState message with the new pose and orientation
    new_state = ModelState()
    new_state.model_name = 'triton_lidar'
    new_state.pose = Pose()
    new_state.pose.position.x = linear_x
    new_state.pose.position.y = linear_y
    new_state.pose.position.z = 0.0
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    new_state.pose.orientation.x = quaternion[0]
    new_state.pose.orientation.y = quaternion[1]
    new_state.pose.orientation.z = quaternion[2]
    new_state.pose.orientation.w = quaternion[3]
    request = SetModelStateRequest()
    request.model_state = new_state

    # Call the service to set the new model state
    result = set_model_state(request)
    return result.success


def laser_callback(msg):
    global zones
    laser = msg.ranges
    zones = []

    zones.append((min(laser[0:15]), laser.index(min(laser[0:15])), 0))
    zones.append((min(laser[355:360]), laser.index(min(laser[355:360])), 0))
    zones.append((min(laser[310:340]), laser.index(min(laser[310:340])), 7))
    zones.append((min(laser[250:290]), laser.index(min(laser[250:290])), 14))
    zones.append((min(laser[210:240]), laser.index(min(laser[210:240])), 21))

def get_state():
    global zones
    min_distance = float(math.inf)
    for t in zones:
        if t[0] < min_distance:
            min_distance = t[0]
            min_distance_angle = t[1]
            min_distance_zone = t[2]

    rospy.loginfo("Minimum Distance = %f", min_distance)
    rospy.loginfo("Minimum Distance Angle = %f", min_distance_angle)
    rospy.loginfo("Minimum Distance Zone = %f", min_distance_zone)

    if min_distance > 0.80:
        state_index = 0+min_distance_zone
    elif min_distance > 0.70 and min_distance <= 0.80:
        state_index = 1+min_distance_zone
    elif min_distance >= 0.60 and min_distance <= 0.70:
        state_index = 2+min_distance_zone
    elif min_distance >= 0.5 and min_distance < 0.6:
        state_index = 3+min_distance_zone
    elif min_distance >= 0.30 and min_distance < 0.50:
        state_index = 4+min_distance_zone
    elif min_distance >= 0.11 and min_distance < 0.30:
        state_index = 5+min_distance_zone
    elif min_distance < 0.11:
        state_index = 6+min_distance_zone
    return state_index, min_distance

def get_action(action):
    if action == 0:
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.0
        return twist

    if action == 1:
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = math.pi/4
        return twist

    if action == 2:
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = -math.pi/4
        return twist

def get_reward(distance):
    if distance > 0.80:
        return -25
    elif distance > 0.70 and distance <= 0.80:
        return -10
    elif distance >= 0.60 and distance <= 0.70:
        return 0
    elif distance >= 0.50 and distance < 0.60:
        return -10
    elif distance >= 0.30 and distance < 0.50:
        return -20
    elif distance >= 0.11 and distance < 0.30:
        return -30
    elif distance < 0.11:
        return -100


def main():
    global q_table
    vel_pub = rospy.Publisher('/triton_lidar/vel_cmd', Twist, queue_size=20)
    scan_sub = rospy.Subscriber("/scan", LaserScan, laser_callback)
    rospy.sleep(1)
    rate = rospy.Rate(20)

    epsilon_zero = 0.9
    d = 0.95
    count = -1
    alpha = 0.3
    gamma = 0.9
    reset_locations = [(-3.25, -2.0, -1.57), (0.0, -3.25, 0.0), (-2.0, 3.25, 3.14), (-2.0, 1.25, 3.14), (2.0, -2.75, 3.14), (1.0, 2.75, 0.0), (2.0, -3.25, 0.0)]
    x ,y, z = -3.25, -2.0, -1.57
    while True:
        if count == 180:
            count = -1
        x, y, z = random.choice(reset_locations)
        reset_robot(x, y, z)
        count += 1
        print("Count is"+str(count))
        episode_reward = 0
        current_state_index = get_state()[0]
        while True:

            # Get Action
            random_number = random.uniform(0, 1)
            if random_number > (epsilon_zero * (d ** (count))):
                current_action_index = q_table[current_state_index].index(max(q_table[current_state_index]))
            else:
                current_action_index = random.randint(0, 2)
            rospy.loginfo("Current State Index = %f", current_state_index)
            rospy.loginfo("Current Action Index = %f", current_action_index)

            # Perform Action
            current_action = get_action(current_action_index)
            vel_pub.publish(current_action)

            # Get Next State
            next_state_index, min_distance = get_state()
            rospy.loginfo("Next State Index = %f", current_state_index)

            # Get Reward
            reward = get_reward(min_distance)
            rospy.loginfo("Next State Reward = %f", reward)
            episode_reward += reward
            if episode_reward < -4000:
                break
            rospy.loginfo("Total episode_reward = %f", episode_reward)

	        # Get Next Action Index
            random_number_two = random.uniform(0, 1)
            if random_number_two > (epsilon_zero * (d ** (count))):
                next_action_index = q_table[next_state_index].index(max(q_table[next_state_index]))
            else:
                next_action_index = random.randint(0, 2)
            #SARSA Update Step
            q_table[current_state_index][current_action_index]  = q_table[current_state_index][current_action_index] + alpha * (reward + (gamma * q_table[next_state_index][next_action_index]) - q_table[current_state_index][current_action_index])
            current_state_index = next_state_index

            print(q_table)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('triton_lidar_wall_follow')
    main()
