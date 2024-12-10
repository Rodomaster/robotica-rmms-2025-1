#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import cos, sin, acos, asin, atan2, sqrt, pi

goal_workspace_position = [0.5, 0.5, 0.6]

class TrajectoryTest(Node):

    def __init__(self, joint_positions):
        super().__init__('trajectory_test')
        topic_name= "/scorbot_trajectory_controller/joint_trajectory"
        self.joint_positions = joint_positions
        self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name,10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self.goal_positions = self.joint_positions
        self.get_logger().info('Controller is running and publishing to topic: {}'.format(topic_name))

    def timer_callback(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)
        

def dot_product(A, B):
    return A[0]*B[0]+A[1]*B[1]+A[2]*B[2]
        
def norm(A):
    return sqrt(dot_product(A, A))
     	
def transpose(A):
    return [[A[0][0],A[1][0],A[2][0]],
    	    [A[0][1],A[1][1],A[2][1]],
     	    [A[0][2],A[1][2],A[2][2]]]
     	
def rotation_matrix_y(theta):
    return [[cos(theta), 0, sin(theta)],
	    [0, 1, 0],
	    [-sin(theta), 0, cos(theta)]]

    	
def matrix_product(A,B):
    if(type(B[0]) == list):
        return [[A[0][0]*B[0][0] + A[0][1]*B[1][0] + A[0][2]*B[2][0], A[0][0]*B[0][1] + A[0][1]*B[1][1] + A[0][2]*B[2][1], A[0][0]*B[0][2] + A[0][1]*B[1][2] + A[0][2]*B[2][2]],
     		[A[1][0]*B[0][0] + A[1][1]*B[1][0] + A[1][2]*B[2][0], A[1][0]*B[0][1] + A[1][1]*B[1][1] + A[1][2]*B[2][1], A[1][0]*B[0][2] + A[1][1]*B[1][2] + A[1][2]*B[2][2]],
     		[A[2][0]*B[0][0] + A[2][1]*B[1][0] + A[2][2]*B[2][0], A[2][0]*B[0][1] + A[2][1]*B[1][1] + A[2][2]*B[2][1], A[2][0]*B[0][2] + A[2][1]*B[1][2] + A[2][2]*B[2][2]]]
    else:
        return [A[0][0]*B[0]+A[0][1]*B[1]+A[0][2]*B[2],A[1][0]*B[0]+A[1][1]*B[1]+A[1][2]*B[2],A[2][0]*B[0]+A[2][1]*B[1]+A[2][2]*B[2]]
        
def vector_difference(a, b):
    return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
     
def getJointPositions(goal_position):
    # Pose inverse kinematic function
    l = [0.352,0.22,0.22,0.1475]
	
    z_1 = [0, 0, 1]
    z_3 = [0, 0, 1]
    p_1_0 = [0, 0, l[0]]
    p_4_p = [0,0,l[2]]

    joint_1 = atan2(goal_position[1],goal_position[0])
    theta_p_0 = atan2(goal_position[2],sqrt(pow(goal_position[0],2) + pow(goal_position[1],2)))

    rotation_matrix_joint_1 = [[cos(joint_1), -sin(joint_1), 0],
			       [-sin(joint_1), cos(joint_1), 0],
			       [0, 0, 1]]
    rotation_matrix_0_p = [[cos(joint_1)*cos(theta_p_0), -sin(joint_1), cos(joint_1)*sin(theta_p_0)],
			   [-cos(theta_p_0)*sin(joint_1),  cos(joint_1), -sin(joint_1)*sin(theta_p_0)],
	     		   [-sin(theta_p_0), 0, cos(theta_p_0)]]
	
    p_4_0 = goal_position + matrix_product(rotation_matrix_0_p, p_4_p) 
    p_4_1 = matrix_product(transpose(rotation_matrix_joint_1), vector_difference(p_4_0, p_1_0))
    p_4_p_0 = vector_difference(p_4_0, goal_position)

    epsilon = acos(dot_product(z_1, p_4_1)/norm(p_4_1))
    alpha = acos((pow(l[1],2) - pow(l[2],2) + pow(norm(p_4_1),2))/(2 * l[1] * norm(p_4_1)))
    beta = asin(norm(p_4_1) * sin(alpha)/l[2])
     	
    joint_2 = epsilon - alpha
    joint_3 = pi - beta
     	
    z_3_0 = matrix_product(matrix_product(rotation_matrix_joint_1, rotation_matrix_y(joint_2 + joint_3)), z_3)
  	
    joint_4 = pi - acos(dot_product(z_3_0,p_4_p_0)/l[3])
    
    #return [joint_1, joint_2, joint_3, joint_4]
    return [0.2, 0.2, 0.2, 0.2]

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher_node = TrajectoryTest(getJointPositions(goal_workspace_position))
    rclpy.spin(trajectory_publisher_node)
    trajectory_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



