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
from numpy.linalg import inv
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def TransMat(alpha, a, d, q, s):
    T = Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                 0,           0,             1]])
    T = T.subs(s)
    return T


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    
    #degree-to-radian
    dtr = pi/180
    
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Define DH param symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        
        # Joint angle symbols
        theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1:7')

  
        # Modified DH params
        s = {alpha0:     0,  a0:      0,  d1:  0.75,
             alpha1: -pi/2,  a1:   0.35,  d2:     0,  q2: q2-pi/2,
             alpha2:     0,  a2:   1.25,  d3:     0,
             alpha3: -pi/2,  a3: -0.054,  d4:  1.50,
             alpha4:  pi/2,  a4:      0,  d5:     0,
             alpha5: -pi/2,  a5:      0,  d6:     0,
             alpha6:     0,  a6:      0,  d7: 0.303,  q7: 0}

        
        # Define Modified DH Transformation matrix
        T0_1 = TransMat(alpha0, a0, d1, q1, s)
        T1_2 = TransMat(alpha1, a1, d2, q2, s)
        T2_3 = TransMat(alpha2, a2, d3, q3, s)
        T3_4 = TransMat(alpha3, a3, d4, q4, s)
        T4_5 = TransMat(alpha4, a4, d5, q5, s)
        T5_6 = TransMat(alpha5, a5, d6, q6, s)
        T6_G = TransMat(alpha6, a6, d7, q7, s)


        # Create individual transformation matrices
        T0_2 = simplify(T0_1 * T1_2)
        T0_3 = simplify(T0_2 * T2_3)
            #T0_4 = simplify(T0_3 * T3_4)
            #T0_5 = simplify(T0_4 * T4_5)
            #T0_6 = simplify(T0_5 * T5_6)
            #T0_G = simplify(T0_6 * T6_G)
        
        R_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                      [sin(pi),  cos(pi), 0, 0],
                      [      0,        0, 1, 0],
                      [      0,        0, 0, 1]])

        R_y = Matrix([[ cos(-pi/2),     0,  sin(-pi/2), 0],
                      [          0,     1,           0, 0],
                      [-sin(-pi/2),     0,  cos(-pi/2), 0],
                      [          0,     0,           0, 1]])
        R_corr = simplify(R_z * R_y)

            #T_total = simplify(T0_G * R_corr)            
        
            
        # Initialize previous theta4 and theta6 to 0. These theta's will be used to find the shortest route for Joint 4 and 6.
        # See Section "theta4 Modification" below 
        pre_theta4 = 0.0
        pre_theta6 = 0.0
        
        
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


            # Calculate joint angles using Geometric IK method
            gam = roll
            bet = pitch
            alp = yaw            
            
            # r's are the componenets of the rotation matrix from base to gripper in URDF frames
            #Transformation matrix using Euler angles
            r11 = cos(alp)*cos(bet)
            r21 = sin(alp)*cos(bet)
            r31 = -sin(bet)
            r12 = cos(alp)*sin(bet)*sin(gam) - sin(alp)*cos(gam)
            r22 = sin(alp)*sin(bet)*sin(gam) + cos(alp)*cos(gam)
            r32 = cos(bet)*sin(gam)
            r13 = cos(alp)*sin(bet)*cos(gam) + sin(alp)*sin(gam)
            r23 = sin(alp)*sin(bet)*cos(gam) - cos(alp)*sin(gam)
            r33 = cos(bet)*cos(gam)
            
            R0_G_rviz = Matrix([[r11, r12, r13, 0.0],
                                [r21, r22, r23, 0.0],
                                [r31, r32, r33, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])
            
                #R0_G_rviz = tf.transformations.euler_matrix(roll, pitch, yaw)


            # Wrist Center Location (URDF frame is used here)
            wx = px - d7 * r11
            wy = py - d7 * r21
            wz = pz - d7 * r31
            
            # theta1	    
            theta1 = atan2(wy,wx)
            theta1 = theta1.subs(s)
            
            
            # theta3
            top = (wz-d1)**2 + (sqrt(wx**2+wy**2)-a1)**2 - a2**2 - d4**2
            bottom = -2*a2*d4
            theta3 = asin(top/bottom)
            theta3 = theta3.subs(s)
		    
		    
		    # theta2
            g = atan2(wz-d1, sqrt(wx**2+wy**2)-a1)
            h = atan2(d4*cos(theta3), a2-d4*sin(theta3))
            theta2 = pi/2 - g - h
            theta2 = theta2.subs(s)
            
            
            # Make theta2 and theta3 lie between 0 to +360 (excluding +360)
            if theta2 < 0:
                theta2 = 2*pi + theta2
            if theta3 < 0:
                theta3 = 2*pi +theta3
            
            # If theta2 and theta3 are outside of the joint limits, find another set of theta's
            if (theta2 > 85*dtr and theta2 < 315*dtr) or (theta3 > 65*dtr and theta3 < 150*dtr):
                theta3 = pi - theta3
		        
                h = atan2(d4*cos(theta3), a2-d4*sin(theta3))
                theta2 = pi/2 - g - h
                theta2 = theta2.subs(s)
                
                # Make the new theta2 and theta3 lie between 0 to +360 (excluding +360)
                if theta2 < 0:
                    theta2 = 2*pi + theta2
                if theta3 < 0:
                    theta3 = 2*pi +theta3
            
            
            # Make theta2 and theta3, which now lie between 0 and +360, conform to the URDF limit format
            if theta2 >= 315*dtr:
                theta2 = theta2 - 2*pi
            if theta3 >= 150*dtr:
                theta3 = theta3 - 2*pi            
            

            # theta4, theta5, and theta6
            T3_0 = T0_3.subs({q1:theta1, q2:theta2, q3:theta3}).T
            R3_G_rviz = T3_0 * R0_G_rviz

            r_22 = R3_G_rviz[1,1]
            r_23 = R3_G_rviz[1,2]
            r_31 = R3_G_rviz[2,0]
            r_11 = R3_G_rviz[0,0]
            r_21 = R3_G_rviz[1,0]

            theta4 = atan2(r_31, -r_11)
            theta5 = atan2(sqrt(r_11**2+r_31**2), r_21)
            theta6 = atan2(r_22, r_23)



            # --------------------------------------------------------------------------------------------------------------
            # If the robot was at its initial pose where all joints are zero,
            # the current theta4 would both be set to pi.
            # Obviously, we want theta4 to be 0 instead of pi at initial pose
            # Therefore, we add pi to theta4 to use another configuration, making theta4 2*pi, or 0.
            # theta5 and theta6 will need to be adjusted due to this new configuration
            #----------------------------------------------------------------------------------------------------------------
            
            
            # theta4 modification -------------------------------------------------------------------------------------------       
            
            # Add pi to use anther configuration. This also make theta4 lie between+0 and +360 (including +360)
            theta4 = theta4 + pi
            
            # Make theta4 lie betwen 0 and +360 (excluding +360)
            if theta4 == 2*pi:
                theta4 = 0                
            
            # If theta4 > 350deg, make theta4 conform to URDF format
            if theta4 > 350*dtr:
                theta4 = theta4 - 2*pi
            
            # The range of joint4 is between -350deg and +350
            # If theta4 is between +10deg and +350deg, there are two possible ways the joint can achieve that
            # The following code selects the shortest route for theta4 by comparing with the previous theta4
            if theta4 >= 10*dtr and theta4 <= 350*dtr:
                theta4_neg = theta4 - 2*pi
                
                angle1 = abs(pre_theta4 - theta4)
                angle2 = abs(pre_theta4 - theta4_neg)
                
                if angle2 < angle1:
                    theta4 = theta4_neg


            #theta5 modification-----------------------------------------------------------------------------------------------
            
            # Adjust theta5 for the configuration of the new theta4
            theta5 = -theta5
            
            
            #theta6 modification-----------------------------------------------------------------------------------------------
            
            # Add pi for the configuration of the new theta4. This also make theta6 lie between +0 and +360 (including +360)
            theta6 = theta6 + pi
            
            # Make theta6 lie between 0 and +360 (excluding +360)
            if theta6 == 2*pi:
                theta6 = 0                
            
            # If theta6 > 350deg, make theta6 conform to URDF format
            if theta6 > 350*dtr:
                theta6 = theta6 - 2*pi
            
            # The range of joint6 is between -350deg and +350
            # If theta6 is between +10deg and +350deg, there are two possible ways the joint can achieve that    
            # The following code select the shortest route for theta6 by comparing with the previous theta6
            if theta6 >= 10*dtr and theta6 <= 350*dtr:
                theta6_neg = theta6 - 2*pi
                
                angle1 = abs(pre_theta6 - theta6)
                angle2 = abs(pre_theta6 - theta6_neg)
                
                if angle2 < angle1:
                    theta6 = theta6_neg
            
            # end (theta6 modification)------------------------------------------------------------------------------------------
            
            
            # setting previous theta4,6 to current theta4,6
            pre_theta4 = theta4
            pre_theta6 = theta6
            
            
            # Evaluating all theta's
            theta1 = theta1.evalf()
            theta2 = theta2.evalf()
            theta3 = theta3.evalf()
            theta4 = theta4.evalf()
            theta5 = theta5.evalf()
            theta6 = theta6.evalf() 
            
		    
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
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
