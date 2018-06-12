#!/usr/bin/env python

import roslib
roslib.load_manifest('simulation_control')
import rospy
import actionlib
import mavros_state
import time

from simulation_control.msg import goto_positionAction, goto_positionGoal, long_grippersAction, long_grippersGoal
from std_msgs.msg import Float32

if __name__ == '__main__':
    rospy.init_node('action_controller')
    mv_state = mavros_state.mavros_state()

while True:
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)

    rospy.loginfo("Taking off")
    goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
    goto_position_client.wait_for_server()
    goto_position_goal = goto_positionGoal()
    goto_position_goal.destination.pose.position.z = 5
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Takeoff succeded")

    # Close legs
    long_grippers_client = actionlib.SimpleActionClient('long_grippers', long_grippersAction)
    long_grippers_client.wait_for_server()
    rospy.loginfo("Moving legs to pickup position")
    long_grippers_goal = long_grippersGoal(grip_rad_goal=Float32(-1.34))
    long_grippers_client.send_goal(long_grippers_goal)
    long_grippers_client.wait_for_result()
    if long_grippers_client.get_result().goal_reached.data:
        rospy.loginfo("Legs moved to pickup position")
    else:
        rospy.loginfo("Error with moving legs to pickup position")
    # /Close legs

    rospy.loginfo("Going to first position")
    goto_position_goal.destination.pose.position.x = -105
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 10
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Arrived at first position.")

    time.sleep(5)

    # Open legs
    long_grippers_client = actionlib.SimpleActionClient('long_grippers', long_grippersAction)
    long_grippers_client.wait_for_server()
    rospy.loginfo("Moving legs to pickup position")
    long_grippers_goal = long_grippersGoal(grip_rad_goal=Float32(0.0))
    long_grippers_client.send_goal(long_grippers_goal)
    long_grippers_client.wait_for_result()
    if long_grippers_client.get_result().goal_reached.data:
        rospy.loginfo("Legs moved to pickup position")
    else:
        rospy.loginfo("Error with moving legs to pickup position")
    # /Open legs

    time.sleep(2)

    rospy.loginfo("Decending on target")
    goto_position_goal.destination.pose.position.x = -105
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 1
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Intended height reached")

    time.sleep(15)

    rospy.loginfo("Decending on target")
    goto_position_goal.destination.pose.position.x = -105
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 0.2
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Intended height reached")

    time.sleep(20)

    # Close legs
    long_grippers_client = actionlib.SimpleActionClient('long_grippers', long_grippersAction)
    long_grippers_client.wait_for_server()
    rospy.loginfo("Moving legs to pickup position")
    long_grippers_goal = long_grippersGoal(grip_rad_goal=Float32(-1.34))
    long_grippers_client.send_goal(long_grippers_goal)
    long_grippers_client.wait_for_result()
    if long_grippers_client.get_result().goal_reached.data:
        rospy.loginfo("Legs moved to pickup position")
    else:
        rospy.loginfo("Error with moving legs to pickup position")
    # /Close legs

    time.sleep(5)

    rospy.loginfo("Going to second position")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = 105
    goto_position_goal.destination.pose.position.z = 5
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Arrived at second position.")

    time.sleep(5)

    rospy.loginfo("Decending")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = 105
    goto_position_goal.destination.pose.position.z = 0.5
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Drop off height reached.")

    time.sleep(3)

    # Open legs
    long_grippers_client = actionlib.SimpleActionClient('long_grippers', long_grippersAction)
    long_grippers_client.wait_for_server()
    rospy.loginfo("Moving legs to drop off position")
    long_grippers_goal = long_grippersGoal(grip_rad_goal=Float32(0.0))
    long_grippers_client.send_goal(long_grippers_goal)
    long_grippers_client.wait_for_result()
    if long_grippers_client.get_result().goal_reached.data:
        rospy.loginfo("Legs moved to drop off position")
    else:
        rospy.loginfo("Error with moving legs to drop off position")
    # /Open legs

    time.sleep(3)

    rospy.loginfo("Going to third position")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 50
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Arrived at third position.")

    time.sleep(10)

    rospy.loginfo("Going to fourth position")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = -105
    goto_position_goal.destination.pose.position.z = 20
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Arrived at fourth position.")

    time.sleep(5)

    rospy.loginfo("Going to fifth position")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 10
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Arrived at fifth position.")

    rospy.loginfo("Landing..")
    mv_state.land(0.0)
    time.sleep(10)
