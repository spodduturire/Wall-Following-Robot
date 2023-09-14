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

q_table = [[-235.7648434150833, -119.43744668612635, -224.78430934539543], [-125.38978580535309, -172.49105339860387, -156.61442211906058], [-149.5424798085129, -153.9663698905482, -101.10889435725339], [-194.89985318251286, -173.89927045769457, -117.9331516903625], [-279.77989328177716, -237.41646483251327, -142.66161964428574], [-422.58870188952835, -432.77885525666704, -416.7791785126422], [-767.564240737533, -399.1992428748392, -768.3201166921185], [-233.6886565132678, -111.24216116070325, -233.84882060060264], [-123.44053371050903, -122.7124514468112, -86.69584569770073], [-96.63703392448531, -143.9723982130352, -153.51244433811493], [-232.06918133436469, -99.77159845849161, -229.84837361435655], [-161.56134413204956, -333.5609271927517, -333.26076494928014], [-515.4277524786487, -305.30203310699943, -520.6763192877405], [-813.0177662929086, -812.9268957225045, -431.23517506866597], [-232.62453251247, -144.5167406960639, -232.84146282257686], [-145.9498942366995, -149.36811579317956, -67.16335621922065], [-113.83899049020567, -105.20792309475102, -54.3954166259675], [-79.70315543082882, -136.987682634064, -174.7115844780245], [-245.07851890807086, -138.41688914199383, -232.9015109257843], [-412.8556382402525, -247.07938062074226, -425.8462990235114], [-730.2460847444095, -738.6063652101394, -628.1996825951826], [-147.21549420184152, -239.5244337288481, -239.63689626820917], [-189.08492609171483, -187.3693642345603, -118.1709953696097], [-132.50501544598094, -131.90750396888217, -41.64268183307205], [-121.6873541344978, -125.82230560332465, -92.40252745761158], [-182.21735692147843, -188.23482052156632, -113.68739426338865], [-201.16954962304587, -240.12188318807847, -247.15587519864772], [0, 0, 0]]

zones = []

#state-action combinations for a single state

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


def main():
    global q_table
    vel_pub = rospy.Publisher('/triton_lidar/vel_cmd', Twist, queue_size=20)
    scan_sub = rospy.Subscriber("/scan", LaserScan, laser_callback)
    rospy.sleep(1)
    rate = rospy.Rate(10)

    current_state_index = get_state()[0]
    main_count = 0
    while True:
        main_count += 1
        if main_count > 1000:
            break
        current_action_index = q_table[current_state_index].index(max(q_table[current_state_index]))

        rospy.loginfo("Current State Index = %f", current_state_index)
        rospy.loginfo("Current Action Index = %f", current_action_index)
        rospy.loginfo("Main Count %f", main_count)

        # Perform Action
        current_action = get_action(current_action_index)
        vel_pub.publish(current_action)

        # Get Next State
        next_state_index, min_distance = get_state()
        if min_distance < 0.11:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            vel_pub.publish(twist)
            break
        current_state_index = next_state_index

        print(q_table)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('triton_lidar_wall_follow')
    reset_robot(-3.0, 2.75, 0.0)
    main()
    reset_robot(2.0, -2.75, 3.14)
    main()
    reset_robot(-3.0, -3.25, 0.0)
    main()
