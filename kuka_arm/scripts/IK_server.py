#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# global variables

# symbols
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")
a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")
d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")
q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8")
r, p, y = symbols("r p y")

# DH parameters
dh = { 
    alpha0:      0, a0:       0,   d1: 0.75,  q1:        q1,
    alpha1: - pi/2, a1:    0.35,   d2: 0,     q2: q2 - pi/2,
    alpha2:      0, a2:    1.25,   d3: 0,     q3:        q3,
    alpha3: - pi/2, a3: - 0.054,   d4: 1.5,   q4:        q4,
    alpha4:   pi/2, a4:       0,   d5: 0,     q5:        q5,
    alpha5: - pi/2, a5:       0,   d6: 0,     q6:        q6,
    alpha6:      0, a6:       0,   d7: 0.303, q7:         0 }

# transformation matrices
T0_E = None

# rotation matrices
R_x = Matrix([[ 1,      0,       0],
              [ 0, cos(r), -sin(r)],
              [ 0, sin(r),  cos(r)]])
R_y = Matrix([[  cos(p), 0, sin(p)],
              [       0, 1,      0],
              [ -sin(p), 0, cos(p)]])
R_z = Matrix([[ cos(y), -sin(y), 0],
              [ sin(y),  cos(y), 0],
              [      0,       0, 1]])
R0_3 = None
R0_E = None

# link lengths
L2_3 = None
L3_5 = None


def tf_matrix(alpha, a, d, q):
    return Matrix([
        [              cos(q),            - sin(q),            0,                a],
        [ sin(q) * cos(alpha), cos(q) * cos(alpha), - sin(alpha), - sin(alpha) * d],
        [ sin(q) * sin(alpha), cos(q) * sin(alpha),   cos(alpha),   cos(alpha) * d],
        [                   0,                   0,            0,                1]])


def initialize_global_variables():
    global T0_E, R_x, R_y, R_z, R0_3, R0_E, L2_3, L3_5

    # Create individual transformation matrices
    T0_1 = tf_matrix(alpha0, a0, d1, q1).subs(dh)
    T1_2 = tf_matrix(alpha1, a1, d2, q2).subs(dh)
    T2_3 = tf_matrix(alpha2, a2, d3, q3).subs(dh)
    T3_4 = tf_matrix(alpha3, a3, d4, q4).subs(dh)
    T4_5 = tf_matrix(alpha4, a4, d5, q5).subs(dh)
    T5_6 = tf_matrix(alpha5, a5, d6, q6).subs(dh)
    T6_E = tf_matrix(alpha6, a6, d7, q7).subs(dh)
    T0_3 = T0_1 * T1_2 * T2_3
    T0_E = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E

    # Create rotation matrices
    R0_3 = T0_3[0:3, 0:3]
    R_crr = R_z.subs({y: pi}) * R_y.subs({p: -pi / 2})
    R0_E = R_z * R_y * R_x * R_crr

    # Calulate link lengths
    L2_3 = dh[a2]
    L3_5 = sqrt(dh[d4]**2 + dh[a3]**2)


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1

    global T0_E, R_z, R0_3, R0_E, L2_3, L3_5, theta4_prev, theta6_prev

    # Initialize service response
    joint_trajectory_list = []
    for x in xrange(0, len(req.poses)):
        # IK code starts here
        joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
        px = req.poses[x].position.x
        py = req.poses[x].position.y
        pz = req.poses[x].position.z

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [req.poses[x].orientation.x, req.poses[x].orientation.y,
                req.poses[x].orientation.z, req.poses[x].orientation.w])

        p0E_0 = Matrix([[px], [py], [pz]])
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        rot0_E = R0_E.subs({r: roll, p: pitch, y: yaw})
        # Wrist-center position
        p0W_0 = p0E_0 - dh[d7] * rot0_E[:,2]

        # Calculate joint angles using Geometric IK method

        theta1 = atan2(p0W_0[1], p0W_0[0])

        # Side lengths of triangle [O2, O3, WC]
        p02_0 = R_z.subs({y: theta1}) * Matrix([[dh[a1]], [0], [dh[d1]]])
        p2w_0 = p0W_0 - p02_0
        side_a = L3_5
        side_b = p2w_0.norm()
        side_c = L2_3

        # Calculate alpha and beta of the triangle
        alpha = acos((side_b * side_b + side_c * side_c - side_a * side_a)
                     / (2 * side_b * side_c))
        beta = acos((side_a * side_a + side_c * side_c - side_b * side_b)
                    / (2 * side_a * side_c))
        # Angle between X2 axis and O2-WC segment
        rad_x2_w = atan2(p0W_0[2] - dh[d1],
                         sqrt(p0W_0[0] * p0W_0[0] + p0W_0[1] * p0W_0[1]) - dh[a1])
        # Angle between X3 axis and O3-WC segment
        rad_x3_w = atan2(- dh[a3], dh[d4])

        theta2 = pi / 2 - alpha - rad_x2_w
        theta3 = pi / 2 - beta - rad_x3_w

        # Rotation between frame 3 and 6
        rot0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
        rot3_6 = rot0_3.inv("LU") * rot0_E

        # Extract theta 4/5/6 from rot3_6
        theta4 = atan2(rot3_6[2, 2], -rot3_6[0, 2])
        theta5 = atan2(sqrt(rot3_6[0, 2] * rot3_6[0, 2] + rot3_6[2, 2] * rot3_6[2, 2]),
                       rot3_6[1, 2])
        theta6 = atan2(-rot3_6[1, 1], rot3_6[1, 0])

        # Calculate end-effector position error
        FK = T0_E.evalf(subs={q1: theta1, q2: theta2, q3:theta3,
                              q4: theta4, q5: theta5, q6: theta6})
        p0E_FK = Matrix([[FK[0, 3]], [FK[1, 3]], [FK[2, 3]]])
        p0E_Err = Matrix([[abs(p0E_FK[0] - px)],
                          [abs(p0E_FK[1] - py)],
                          [abs(p0E_FK[2] - pz)]])
        print ("End effector offset: ", p0E_Err.norm())

        # Populate response for the IK request
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

    rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    initialize_global_variables()
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
