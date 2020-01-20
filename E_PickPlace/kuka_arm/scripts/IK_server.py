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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        d1,     d2,     d3,     d4,     d5,     d6,     d7      = symbols('d1:8')
        a0,     a1,     a2,     a3,     a4,     a5,     a6      = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6  = symbols('alpha0:7')
        q1,     q2,     q3,     q4,     q5,     q6,     q7      = symbols('q1:8')

        # Create Modified DH parameters
        DH_table = {
            alpha0: 0, 	        a0: 0, 		d1: 0.75, 	q1: q1,
            alpha1: -pi / 2.,   a1: 0.35,	d2: 0, 		q2: -pi / 2. + q2,
            alpha2: 0, 	        a2: 1.25, 	d3: 0, 		q3: q3,
            alpha3: -pi/2.,     a3: -0.054, d4: 1.5, 	q4: q4,
            alpha4: pi/2, 	    a4: 0, 		d5: 0, 		q5: q5,
            alpha5: -pi/2.,     a5: 0, 		d6: 0, 		q6: q6,
            alpha6: 0, 	        a6: 0, 		d7: 0.303, 	q7: 0
            }
        # Define Modified DH Transformation matrix
        def get_transformation_matrix(alpha, a, d, q):
            TFMat = Matrix([
                [cos(q), 		        -sin(q), 		        0.0, 		    a               ],
                [sin(q) * cos(alpha), 	cos(q) * cos(alpha), 	-sin(alpha), 	-sin(alpha) * d ],
                [sin(q) * sin(alpha), 	cos(q) * sin(alpha), 	cos(alpha), 	cos(alpha) * d  ],
                [0.0,			        0.0,			        0.0,		    1.0             ]
                ])
            return TFMat
        
        # Create individual transformation matrices
        TFMat_0t1 = get_transformation_matrix(alpha0, a0, d1, q1).subs(DH_table)
        TFMat_1t2 = get_transformation_matrix(alpha1, a1, d2, q2).subs(DH_table)
        TFMat_2t3 = get_transformation_matrix(alpha2, a2, d3, q3).subs(DH_table)
        TFMat_3t4 = get_transformation_matrix(alpha3, a3, d4, q4).subs(DH_table)
        TFMat_4t5 = get_transformation_matrix(alpha4, a4, d5, q5).subs(DH_table)
        TFMat_5t6 = get_transformation_matrix(alpha5, a5, d6, q6).subs(DH_table)
        TFMat_6tE = get_transformation_matrix(alpha6, a6, d7, q7).subs(DH_table)

        TFMat_0tE = simplify(TFMat_0t1 * TFMat_1t2 * TFMat_2t3 * TFMat_3t4 * TFMat_4t5 * TFMat_5t6 * TFMat_6tE)

        r, p, y = symbols('r p y')

        rotation_x = Matrix([
            [1.0,   0.0,      0.0     ],
            [0.0,   cos(r),   -sin(r) ],
            [0.0,   sin(r),   cos(r)  ]
            ])

        rotation_y = Matrix([
            [cos(p), 	0.0 , 	sin(p)  ],
            [0.0, 		1.0, 	0.0     ],
            [-sin(p), 	0.0, 	cos(p)  ]
            ])

        rotation_z = Matrix([
            [cos(y),    -sin(y),    0.0],
            [sin(y),    cos(y),     0.0],
            [0.0,       0.0,        1.0]
            ])
	
        rotation_E = simplify(rotation_z * rotation_y * rotation_x)
        rotation_err = rotation_z.subs(y, radians(180)) * rotation_y.subs(p, radians(-90))
        rotation_E = simplify(rotation_E * rotation_err)

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

            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                req.poses[x].orientation.z, req.poses[x].orientation.w])

            rotation_E = rotation_E.subs({'r': roll, 'p': pitch, 'y': yaw})

            pos_end_effector = Matrix([
                [px], 
                [py], 
                [pz]
                ])

            wrist_center = pos_end_effector - 0.303 * rotation_E[:,2]
            theta1 = atan2(wrist_center[1], wrist_center[0])

            A = 1.501
            B = sqrt(pow(sqrt(wrist_center[0] ** 2 + wrist_center[1] ** 2) - 0.35, 2) + pow((wrist_center[2] - 0.75), 2))
            C = 1.25

            angle_a = acos((B ** 2 + C ** 2 - A ** 2) / (2.0 * B * C))
            angle_b = acos((A ** 2 + C ** 2 - B ** 2) / (2.0 * A * C))
            angle_c = acos((A ** 2 + B ** 2 - C ** 2) / (2.0 * A * B))

            theta2 = pi / 2.0 - angle_a - atan2(wrist_center[2] - 0.75, sqrt(wrist_center[0] ** 2 + wrist_center[1] ** 2) - 0.35)
            theta3 = pi / 2.0 - (angle_b + 0.036)

            rotation_0t3 = TFMat_0t1[0: 3, 0: 3] * TFMat_1t2[0: 3, 0: 3] * TFMat_2t3[0: 3, 0: 3]
            rotation_0t3 = rotation_0t3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})
            rotation_3t6 = rotation_0t3.transpose() * rotation_E

            theta4 = atan2(rotation_3t6[2, 2], -rotation_3t6[0, 2])
            theta5 = atan2(sqrt(rotation_3t6[0, 2] ** 2 + rotation_3t6[2, 2] ** 2), rotation_3t6[1, 2])
            theta6 = atan2(-rotation_3t6[1, 1], rotation_3t6[1, 0])

            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
	IK_server()
