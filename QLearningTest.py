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

q_table = [[-161.3261106564881, -215.427099148481, -199.79828390805], [-143.0969313995053, -150.4010472913177, -138.84300133401166], [-179.621994682548, -180.8663152927668, -100.2425788379275], [-284.9996949433888, -278.7084189014754, -251.26414206983793], [-402.2093848182891, -419.91678405527136, -426.92594177593156], [-560.9616368729719, -621.69022235502, -506.74438785991896], [-764.0675003591481, -444.8787234687593, -767.3132804473131], [-204.0256017667103, -184.65331383029738, -186.7219891239538], [-110.50044092421585, -76.42392174532837, -118.37923362422069], [-162.49327777291538, -86.80111525611758, -158.20881243257548], [-178.67495301166466, -251.34574462895634, -257.42122008914623], [-445.83914068004157, -187.1948003258817, -414.6905624416454], [-604.1968999766325, -599.9189304631627, -606.6545484553067], [-665.1950041360806, -733.0471808720314, -731.8701622693741], [-223.5470606222673, -228.06880064042443, -201.11860853195782], [-135.30520424979346, -143.93394893567532, -90.74011117885226], [-48.418124633191646, -73.41099209973765, -111.50929740610421], [-177.5336578836394, -77.10051609351066, -181.0847972433728], [-217.52512629975027, -169.0154248012462, -236.41522837195618], [-488.54140885466194, -401.4101169206139, -492.05038649551216], [-530.1792518575284, -651.6463263976433, -652.4772439638732], [-226.71142146913675, -226.18877804677436, -205.99685885598564], [-150.21569143721499, -181.84248663825818, -161.87562435876538], [-89.23208441584761, -98.92442581425625, -106.58171903120562], [-92.49915513850912, -85.45194741545326, -104.30124972262341], [-132.8994404092555, -162.7462798366776, -183.09304897597565], [-175.91777834051524, -248.57020969126557, -168.12283546710174], [0, 0, 0]]

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
    reset_robot(-3.20, -2.0, -1.57)
    main()
    reset_robot(-1.0, 3.5, 3.14)
    main()
    reset_robot(0.5, -2.5, 3.14)
    main()
    reset_robot(0.0, 2.75, 0.0)
    main()
