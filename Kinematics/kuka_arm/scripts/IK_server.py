# Import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# Define function to create individual transformation matrices
def T(alpha, a, d, q):    
    M = Matrix([[            cos(q),           -sin(q),           0,             a],
                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
    return M

# Define functions for Rotation Matrices about x, y, and z given specific angle
def rot_x(q):
    R_x = Matrix([[      1,       0,       0],
                  [      0,  cos(q), -sin(q)],
                  [      0,  sin(q),  cos(q)]])
    return R_x

def rot_y(q):              
    R_y = Matrix([[ cos(q),       0,  sin(q)],
                  [      0,       1,       0],
                  [-sin(q),       0,  cos(q)]])
    return R_y

def rot_z(q):    
    R_z = Matrix([[ cos(q), -sin(q),       0],
                  [ sin(q),  cos(q),       0],
                  [      0,       0,       1]])
    return R_z

# Main functions
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

    if len(req.poses) < 1:
        print "No valid poses received"
        return -1

    else:
        # Create symbols for joint angle & DH parameters
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')                        
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') 
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') 
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') 

        # Define Modified DH Transformation matrix
        s = {alpha0:     0, a0:      0, d1:  0.75, 
             alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,  
             alpha2:     0, a2:   1.25, d3:     0,
             alpha3: -pi/2, a3: -0.054, d4:   1.5,
             alpha4:  pi/2, a4:      0, d5:     0,
             alpha5: -pi/2, a5:      0, d6:     0,
             alpha6:     0, a6:      0, d7: 0.303, q7: 0}

        # Create individual transformation matrices
        T0_1 = T(alpha0, a0, d1, q1)
        T0_1 = T0_1.subs(s)

        T1_2 = T(alpha1, a1, d2, q2)
        T1_2 = T1_2.subs(s)
        T0_2 = simplify(T0_1 * T1_2)
        #print("T0_2")

        T2_3 = T(alpha2, a2, d3, q3)
        T2_3 = T2_3.subs(s)
        T0_3 = simplify(T0_2 * T2_3)
        #print("T0_3")

        T3_4 = T(alpha3, a3, d4, q4)
        T3_4 = T3_4.subs(s)
        T0_4 = simplify(T0_3 * T3_4)
        #print("T0_4")

        T4_5 = T(alpha4, a4, d5, q5)
        T4_5 = T4_5.subs(s)
        T0_5 = simplify(T0_4 * T4_5)
        #print("T0_5")

        T5_6 = T(alpha5, a5, d6, q6)
        T5_6 = T5_6.subs(s)
        T0_6 = simplify(T0_5 * T5_6)
        #print("T0_6")

        T6_7 = T(alpha6, a6, d7, q7)
        T6_7 = T6_7.subs(s)
        T0_7 = simplify(T0_6 * T6_7) # transform from base link to end effector
        #print("T0_7")

        # Correction needed to account of orientation difference between definition of gripper_link in URDF vs. DH convention - 
        # First rotate around z-axis by pi then rotate around y-axis by -pi/2
        R_z = rot_z(pi)
        T_z = R_z.row_join(Matrix([[0], [0], [0]]))
        T_z = T_z.col_join(Matrix([[0, 0, 0, 1]])) 

        R_y = rot_y(-pi/2)
        T_y = R_y.row_join(Matrix([[0], [0], [0]]))
        T_y = T_y.col_join(Matrix([[0, 0, 0, 1]])) 

        R_corr = simplify(T_z * T_y)

        # Total homogeneous transformation between base_link and gripper_link with correction
        T_total = simplify(T0_7 * R_corr) 
        #print("T_total")

        # Initialize service response
        joint_trajectory_list = []

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
            
            # Extract end-effector position and orientation from request
            # px, py, pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            P_EE = Matrix([[px],[py],[pz]])
            
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                                 req.poses[x].orientation.z, req.poses[x].orientation.w])
 
            # Since the overall RPY (Roll Pitch Yaw) rotation between base_link and gripper_link must be 
            # equal to the product of individual rotations between respective links: R0_6 = Rrpy    
            R_roll = rot_x(roll)
            R_pitch = rot_y(pitch)
            R_yaw = rot_z(yaw)
            R0_6 = simplify(R_roll * R_pitch * R_yaw)

            # Calculate the location of the spherial wrist center
            P_WC = simplify(P_EE - 0.303 * R0_6 * Matrix([[1],[0],[0]]))
            j5 = P_WC

            # Solve for first 3 joint angles                   
            theta1 = atan2(j5[1], j5[0])

            j2_0 = [0.35, 0, 0.75]
            j3_0 = [0.35, 0, 2]
            j5_0 = [1.8499, 0, 1.9465] 

            j2 = [j2_0[0] * cos(theta1), j2_0[0] * sin(theta1), j2_0[2]]

            
            L2_5_X = j5[0] - j2[0]
            L2_5_Y = j5[1] - j2[1]
            L2_5_Z = j5[2] - j2[2]
            L2_5 = sqrt(L2_5_X**2 + L2_5_Y**2 + L2_5_Z**2)

            L2_3__0 = 1.25

            L3_5_X__0 = J5__0[0] - J3__0[0]
            L3_5_Z__0 = J5__0[2] - J3__0[2]
            L3_5__0 = sqrt(L3_5_X__0**2 + L3_5_Z__0**2)

            #D = cos(theta)
            D = (L2_5**2 - L2_3__0**2 - L3_5__0**2) / -(2 * L2_3__0 * L3_5__0)

            theta3_internal = atan2(sqrt(1-D**2), D)
            theta3 = pi / 2 - (atan2(sqrt(1-D**2), D) - atan2(L3_5_Z__0, L3_5_X__0))
            theta3_2 = pi / 2 - (atan2(-sqrt(1-D**2), D) - atan2(L3_5_Z__0, L3_5_X__0))
            #q3_1 = atan2(sqrt(1-D**2), D)
            #q3_2 = atan2(-sqrt(1-D**2), D) 

            #q2 = atan2(L2_5_Z, L2_5_X) - atan2(L3_5__0 * sin(pi - q3_internal), L2_3__0 + L3_5__0 * cos(pi - q3_internal))
            m1 = L3_5__0 * sin(theta3_internal)
            m2 = L2_3__0 - L3_5__0 * cos(theta3_internal)
            b2 = atan2(m1,m2)
            b1 = atan2(J5[2]-J2[2], sqrt((J5[0]-J2[0])**2 + (J5[1]-J2[1])**2))
            theta2 = pi / 2 - b2 - b1

            # Evaluate with calculated q1, q2 & q3
            R0_3 = T0_3[0:3, 0:3] 
            R0_3_num = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            #rospy.loginfo("R0_3_num", R0_3_num)

            #calculate inverse of R0_3
            R0_3_num_inv = R0_3_num ** -1

            R3_6 = R0_3_num_inv * R0_6
            
            # the orientation of the end effector is know from ROS.         
            theta4 = atan2(R3_6[2, 1], R3_6[2, 2])
            theta5 = atan2(-R3_6[2, 0], sqrt(R3_6[0, 0]*R3_6[0, 0] + R3_6[1, 0]*R3_6[1, 0]))
            theta6 = atan2(R3_6[1, 0], R3_6[0, 0])

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
