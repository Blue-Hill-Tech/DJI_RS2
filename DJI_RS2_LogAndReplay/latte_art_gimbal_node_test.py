#!/usr/bin/env python3.8
import sys
import os

path_to_remove = [os.getenv("HOME") + '/Dropbox/bluehill_deploy/lib/bluehill/python3.5/dist-packages']
sys_path = set(sys.path)

for path in path_to_remove:
    if path in sys_path:
        sys.path.remove(path)

import numpy as np
import rospy
import signal
import time


from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


global current_joint_state
current_joint_state = JointState()

def joint_state_callback(data):
    global current_joint_state
    current_joint_state = data
    #print(str(data))

if __name__ == "__main__":
    rospy.init_node("latte_art_gimbal_node_test")

    # Setup publishers and subscribers

    max_rotation = 90 # degrees

    sub_joint_state = rospy.Subscriber("/latte_art_stage/joint_state", JointState, callback=joint_state_callback, queue_size=1)

    rospy.loginfo("Testing Latte Art Gimbal Node, executing points, please wait...")
    pub_point = rospy.Publisher("/latte_art_stage/execute_point", JointTrajectoryPoint, queue_size=0)
    pub_point.publish(JointTrajectoryPoint(positions=[0.0, 0.0, 0.0], time_from_start=rospy.Duration(0.0)))
    time.sleep(1)
    for angle in range(0, max_rotation):
        target = angle
        pub_point.publish(JointTrajectoryPoint(positions=[target, 0.0, 0.0], time_from_start=rospy.Duration(0.0)))
        time.sleep(0.1)
        rospy.loginfo("target: " + str(target) + " current: "+ str(np.round(np.array(current_joint_state.position[0]) * 10) / 10.0))

    for angle in range(0, max_rotation):
        target = max_rotation - angle
        pub_point.publish(JointTrajectoryPoint(positions=[target, 0.0, 0.0], time_from_start=rospy.Duration(0.0)))
        time.sleep(0.1)
        rospy.loginfo("target: " + str(target) + " current: " + str(
            np.round(np.array(current_joint_state.position[0]) * 10) / 10.0))


    rospy.loginfo("Testing Latte Art Gimbal Node, executing trajectory, please wait...")
    pub_trajectory = rospy.Publisher("/latte_art_stage/execute_trajectory", JointTrajectory, queue_size=0)
    pub_trajectory.publish(JointTrajectory())
    time.sleep(1)
    trajectory = JointTrajectory()
    trajectory.joint_names = ['yaw', 'pitch', 'roll']
    trajectory.points = []

    for angle in range(0, max_rotation):
        trajectory.points.append(JointTrajectoryPoint(positions=[angle, 0.0, 0.0], time_from_start=rospy.Duration(len(trajectory.points) / 10.0)))
    for angle in range(0, max_rotation):
        trajectory.points.append(JointTrajectoryPoint(positions=[max_rotation - angle, 0.0, 0.0], time_from_start=rospy.Duration(len(trajectory.points) / 10.0)))

    pub_trajectory.publish(trajectory)

    for angle in range(0, max_rotation):
        rospy.loginfo(np.round(np.array(current_joint_state.position)*10)/10.0)
        time.sleep(0.2)

    time.sleep(1)

    rospy.loginfo("Test of latte art gimbal is complete.")
    os._exit(os.EX_OK)


