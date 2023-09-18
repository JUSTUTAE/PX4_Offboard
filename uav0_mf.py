#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

target_distance=0
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("uav0")

    state_sub = rospy.Subscriber("uav0/mavros/state", State, callback=state_cb)

    local_pos_pub = rospy.Publisher("uav0/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/uav0/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("uav0/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/uav0/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("uav0/mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    target_distance= rospy.get_param("uav0_mf/target_distance", 1.0)

    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    target_distance=target_distance
    speed = 0.2  # Desired speed of movement (in meters per second)

    start_time = rospy.Time.now()
    current_time = start_time

    reached_target = False  # Flag to indicate if the target distance has been reached

    while not rospy.is_shutdown() and not reached_target:
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("uav0 armed")
                last_req = rospy.Time.now()

        current_time = rospy.Time.now()
        elapsed_time = current_time - start_time

        displacement = speed * elapsed_time.to_sec()  # 변위 계산: 변위 = 속도 × 시간 (직선 운동 방정식)
        pose.pose.position.x = displacement

        if displacement >= target_distance:
            reached_target = True
            pose.pose.position.x = target_distance  # 최종 위치에 도달하면 세트포인트를 최종 위치로 설정

        local_pos_pub.publish(pose)
        rate.sleep()

    while not rospy.is_shutdown():
        local_pos_pub.publish(pose)
        rate.sleep()
