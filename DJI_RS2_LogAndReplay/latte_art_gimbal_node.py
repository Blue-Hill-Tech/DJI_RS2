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
import math

from bluehill.msg import Diagnostics
from dji_controller import DJIController
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


global heart_beat_last_time
heart_beat_last_time = 0

global run
run = True
global ready
ready = False
global deviate_from_path
deviate_from_path = False

global gimbal
gimbal = DJIController("can0")
gimbal.bus.flush_tx_buffer()

dt = 0.1  # 100ms between each update
kp = np.array([2, 2, 2]) * 1
ki = np.array([0.1, 0.1, 0.1]) * 1
kd = np.array([0.5, 0.5, 0.5]) * 1


#rospy.loginfo("Connected: ", servo.read_model())

MAX_ERROR_COUNT = 10

from multiprocessing import Lock

mutex = Lock()

class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.previous_error = np.array([0.0, 0.0, 0.0])
        self.integral = np.array([0.0, 0.0, 0.0])

    def update(self, desired_pose, current_pose):
        error = desired_pose - current_pose
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


def execute_trajectory_callback(joint_trajectory):
    global ready, deviate_from_path
    rospy.loginfo("Received new trajectory msg")

    pid_controller = PID(kp, ki, kd, dt)

    if ready:
        start_time = rospy.get_time()
        prev_target_point = np.array(joint_trajectory.points[0].positions)
        for target_point in joint_trajectory.points:
            sleep_time = target_point.time_from_start.to_sec() - rospy.get_time() + start_time
            if sleep_time < 0:
                continue
            # rospy.logwarn("Sleeping for " + str(sleep_time))
            rospy.sleep(target_point.time_from_start.to_sec() - rospy.get_time() + start_time)
            #rospy.sleep(0.1)
            current_point = np.array([gimbal.yaw, gimbal.pitch, gimbal.roll])

            rospy.loginfo("prev_target: " + str(prev_target_point) + " current: " + str(current_point))
            deviation = np.linalg.norm(prev_target_point - current_point)
            if deviation > 20:
                deviate_from_path = True
                rospy.logwarn("Path deviation detected " + str(deviation))

            # compute target angle use PID controller.
            controll_point = pid_controller.update(target_point.positions, current_point)

            controll_point_msg = JointTrajectoryPoint()
            #controll_point_msg.positions = controll_point
            controll_point_msg.positions = target_point.positions
            set_angle_callback(controll_point_msg, display_log=False)
            prev_target_point = np.array(target_point.positions)

        # return to zero position after trajectory execution
        controll_point_msg = JointTrajectoryPoint()
        controll_point_msg.positions = [0, 0, 0]
        set_angle_callback(controll_point_msg, display_log=False)
        time.sleep(1.0)
    else:
        rospy.logwarn("Servo is not ready yet!")


def set_angle_callback(joint_point_msg, display_log=True):
    global servo, ready, mutex

    ypr = joint_point_msg.positions

    if display_log:
        rospy.loginfo("Received new joint angle msg: " + str(ypr))

    if len(ypr) != 3:
        rospy.logerr("Gimbal joint angles needs [yaw, pitch, roll] as input, but received: " + str(ypr))
        return

    if ready:
        #ypr = (ypr % 360)
        #if ypr > 180:
        #    ypr = -(360 - ypr)
        with mutex:
            try:
                gimbal.set_ypr(ypr, dt)
            except:
                rospy.logwarn("Gimbal failed to rotate to desired angles: " + str(ypr))

    else:
        rospy.logwarn("Gimbal is not ready yet!")


def rosShutdown(a=0, b=0):
    global servo, run

    if run:
        run = False
        time.sleep(0.5)

        rospy.loginfo("Gimbal close successful!")

        time.sleep(0.25)

        try:
            sys.exit()  # this always raises SystemExit
        except SystemExit:
            rospy.loginfo("sys.exit() worked as expected")
        except:
            rospy.loginfo("Something went horribly wrong")  # some other exception got raise


if __name__ == "__main__":
    node_name = "latte_art_gimbal_node"
    rospy.init_node(node_name)

    # Setup publishers and subscribers
    pub = rospy.Publisher("latte_art_stage/joint_state", JointState, queue_size=1)
    pub_diagnostics = rospy.Publisher("/bluehill/diagnostic_message", Diagnostics, queue_size=1)
    protective_stop_pub = rospy.Publisher("latte_art_stage/protective_stop", Empty, queue_size=1)

    rospy.Subscriber("latte_art_stage/execute_point", JointTrajectoryPoint, set_angle_callback)
    rospy.Subscriber("latte_art_stage/execute_trajectory", JointTrajectory, execute_trajectory_callback)

    signal.signal(signal.SIGTERM, rosShutdown)
    rospy.on_shutdown(rosShutdown)

    heart_beat_last_time = rospy.get_time()
    joint_state_last_time = rospy.get_time()

    time.sleep(1)

    # go to starting pose
    # gimbal.set_ypr([0, 0, 0], dt)
    # time.sleep(1.0)

    ready = True
    error_count = 0

    rospy.loginfo(node_name + " started!")

    while run:
        if rospy.get_time() - joint_state_last_time > 0.02:
            joints = JointState()
            joints.name = ['yaw', 'pitch', 'roll']
            encoder_read_success = False
            with mutex:
                try:
                    joints.position = gimbal.position.copy()
                    joint_state_last_time = rospy.get_time()
                    joints.header.stamp = rospy.Time.from_sec(gimbal.position_time)
                    encoder_read_success = True
                except:
                    rospy.logwarn("Servo failed to read encoder")
                    error_count = error_count + 1
            if error_count > MAX_ERROR_COUNT:
                run = False
            if encoder_read_success:
                error_count = 0
                pub.publish(joints)

        if deviate_from_path:
            deviate_from_path = False
            protective_stop_pub.publish(Empty())

        if rospy.get_time() - heart_beat_last_time > 1:
            heart_beat_last_time = rospy.get_time()
            disgonstic_msg = Diagnostics()
            disgonstic_msg.header.stamp = rospy.Time.now();
            disgonstic_msg.name = node_name
            disgonstic_msg.severity = Diagnostics.HEARTBEAT
            pub_diagnostics.publish(disgonstic_msg)

        time.sleep(0.005)

    rospy.loginfo(node_name + "close successful!")


    try:
        sys.exit()  # this always raises SystemExit
    except SystemExit:
        rospy.loginfo("sys.exit() worked as expected")
    except:
        rospy.loginfo("Something went horribly wrong")  # some other exception got raise

    os._exit(os.EX_OK)
