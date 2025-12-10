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
from rmd_servo import RMD_S_Servo
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

global heart_beat_last_time
heart_beat_last_time = 0

global run
run = True
global ready
ready = False
global deviate_from_path
deviate_from_path = False

global current_js
current_js = 0.0

latte_art_stage_files = ['/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0',
                         '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0']
latte_art_stage_port = None

for port in latte_art_stage_files:
    if os.path.exists(port):
        latte_art_stage_port = port
if latte_art_stage_port is None:
    rospy.logerr('Latte Art Stage is not connected. Exiting...')
    time.sleep(1)
    exit()

global servo
servo = RMD_S_Servo(latte_art_stage_port)
rospy.loginfo("Connected: ", servo.read_model())

MAX_ERROR_COUNT = 10

from multiprocessing import Process, Lock

mutex = Lock()


class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, desired_pose, current_pose):
        error = desired_pose - current_pose
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


def execute_trajectory_callback(joint_trajectory):
    global ready
    rospy.loginfo("Received new trajectory msg")
    if ready:
        start_time = rospy.get_time()
        prev_point = joint_trajectory.points[0].positions[0]
        angle = Float32
        for point in joint_trajectory.points:
            sleep_time = point.time_from_start.to_sec() - rospy.get_time() + start_time
            if sleep_time < 0:
                continue
            # rospy.logwarn("Sleeping for " + str(sleep_time))
            rospy.sleep(point.time_from_start.to_sec() - rospy.get_time() + start_time)
            angle.data = point.positions[0]
            if math.fabs(prev_point - current_js) > 10:
                deviate_from_path = True
                rospy.logwarn("Path deviation detected " + str(math.fabs(prev_point - current_js)))
            set_angle_callback(angle)
            prev_point = point.positions[0]
        # return to zero position after trajectory execution
        angle.data = 0.0
        set_angle_callback(angle)
        time.sleep(0.005)
    else:
        rospy.logwarn("Servo is not ready yet!")


def execute_trajectory_callback_with_pid(joint_trajectory):
    global ready
    rospy.loginfo("Received new trajectory msg")

    kp = 4
    ki = 0.5
    kd = 0.5
    dt = 0.1

    pid_controller = PID(kp, ki, kd, dt)

    if ready:
        start_time = rospy.get_time()
        prev_point = joint_trajectory.points[0].positions[0]
        angle = Float32
        for point in joint_trajectory.points:
            sleep_time = point.time_from_start.to_sec() - rospy.get_time() + start_time
            if sleep_time < 0:
                continue
            # rospy.logwarn("Sleeping for " + str(sleep_time))
            rospy.sleep(point.time_from_start.to_sec() - rospy.get_time() + start_time)

            if math.fabs(prev_point - current_js) > 10:
                deviate_from_path = True
                rospy.logwarn("Path deviation detected " + str(math.fabs(prev_point - current_js)))

            # compute target angle use PID controller.
            angle.data = pid_controller.update(point.positions[0], current_js)
            set_angle_callback(angle)
            prev_point = point.positions[0]
        # return to zero position after trajectory execution
        angle.data = 0.0
        set_angle_callback(angle)
        time.sleep(0.005)
    else:
        rospy.logwarn("Servo is not ready yet!")


def set_angle_callback(angle):
    global servo, ready, mutex
    if ready:
        rotation_angle = (angle.data % 360)
        if rotation_angle > 180:
            rotation_angle = -(360 - rotation_angle)
        with mutex:
            try:
                servo.move_closed_loop_angle(rotation_angle)
            except:
                rospy.logwarn("Servo failed to rotate to desired angle: " + str(rotation_angle))

    else:
        rospy.logwarn("Servo is not ready yet!")


def rosShutdown(a=0, b=0):
    global servo, run

    if run:
        run = False
        time.sleep(0.5)
        servo.stop()
        servo.clear_errors()
        servo.shutdown()

        rospy.loginfo("Servo close successful!")

        time.sleep(0.25)

        sys.exit(0)

if __name__ == "__main__":
    rospy.init_node("latte_art_stage_node")
    pub = rospy.Publisher("latte_art_stage/joint_state", JointState, queue_size=1)
    pub_diagnostics = rospy.Publisher("/bluehill/diagnostic_message", Diagnostics, queue_size=1)
    protective_stop_pub = rospy.Publisher("latte_art_stage/protective_stop", Empty, queue_size=1)

    rospy.Subscriber("latte_art_stage/joint_0/angle", Float32, set_angle_callback)
    rospy.Subscriber("latte_art_stage/execute_trajectory", JointTrajectory, execute_trajectory_callback)

    signal.signal(signal.SIGTERM, rosShutdown)
    rospy.on_shutdown(rosShutdown)

    heart_beat_last_time = rospy.get_time()
    joint_state_last_time = rospy.get_time()
    servo.shutdown()
    servo.clear_errors()
    pi_params = servo.read_pi_parameters()

    pi_params.torque_kp = 2
    pi_params.torque_ki = 2
    pi_params.speed_kp = 20
    pi_params.speed_ki = 20
    pi_params.angle_kp = 20
    pi_params.angle_ki = 20

    servo.write_pi_parameters_ram(pi_params)
    time.sleep(1)
    servo.enable_movement()

    # go to starting pose
    servo.move_closed_loop_angle(0)

    ready = True
    error_count = 0
    while run:
        if rospy.get_time() - joint_state_last_time > 0.02:
            joints = JointState()
            joints.name = ['j0']
            encoder_read_success = False
            with mutex:
                try:
                    joints.position = [servo.read_encoder()]
                    current_js = joints.position[0]
                    joint_state_last_time = rospy.get_time()
                    joints.header.stamp = rospy.Time.now();
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
            disgonstic_msg.name = 'latte_art_stage_node'
            disgonstic_msg.severity = Diagnostics.HEARTBEAT
            pub_diagnostics.publish(disgonstic_msg)

        time.sleep(0.005)

    servo.stop()
    servo.clear_errors()
    servo.shutdown()
    rospy.loginfo("Servo close successful!")


