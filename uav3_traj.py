#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math

r = 0
theta = 0
count = 0
wn = 0

current_state = State()


def state_cb(msg):
    global current_state
    current_state = msg
    
def position_cb(msg):
    global current_position
    current_position = msg 


if __name__ == "__main__":
    rospy.init_node("uav3")

    state_sub = rospy.Subscriber("uav3/mavros/state", State, callback=state_cb)
    position_sub = rospy.Subscriber("uav3/mavros/local_position/pose", PoseStamped, callback= position_cb)

    local_pos_pub = rospy.Publisher("uav3/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/uav3/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("uav3/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/uav2/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("uav3/mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    wn = rospy.get_param("pub_setpoints_traj/wn", 1.0)
    r = rospy.get_param("pub_setpoints_traj/r", 1.0)

    # Wait for Flight Controller connection
    while not rospy.is_shutdown() and current_state.connected:
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    i = 100
    while not rospy.is_shutdown() and i > 0:
        local_pos_pub.publish(pose)
        rate.sleep()
        i -= 1

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("uav3 armed")

                last_req = rospy.Time.now()
        theta = wn * count * 0.05

        pose.pose.position.x = r * math.sin(theta)
        pose.pose.position.y = r * math.cos(theta)
        pose.pose.position.z = 2

        count += 1

        local_pos_pub.publish(pose)
        rate.sleep()
