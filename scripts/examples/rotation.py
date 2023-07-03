#! /usr/bin/env python

import math
import time
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    first = True

    while(not rospy.is_shutdown()):
        # First set the mode to offboard (refer to PX4 Flight Modes)
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()

        # Armed the vehicle
        elif(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle armed")

            time.sleep(5) # Wait for 5 seconds after arming for takeoff

            last_req = rospy.Time.now()

        # Move the vehicle
        elif(current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if first:
                pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
                first = False
            else:
                if pose.pose.orientation == Quaternion(*tf.transformations.quaternion_from_euler(0, 0, math.pi/2)):
                    pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
                    rospy.loginfo("Rotating Back")
                else:
                    pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, math.pi/2))

                    rospy.loginfo("Roatating")

            last_req = rospy.Time.now()


        local_pos_pub.publish(pose)

        rate.sleep()

