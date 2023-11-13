#!/usr/bin/env python3

import time
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
import rospy
import numpy as np
import actionlib
import sys, select, termios, tty 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose
from geomagic_control.msg import OmniFeedback
from math import sin, cos
import math

def Call():
    global t
    t=0
    rospy.init_node('joint_limit', anonymous=True)
    rate = rospy.Rate(100) 
    rospy.Subscriber('/Geomagic1/joint_states', JointState, main, queue_size=1)
    #rospy.Subscriber('/Geomagic1/end_effector_pose', JointState, end_effector_check, queue_size=1)
    rospy.spin()

def end_effector_check(data):
    print(data)
    
def gradient_jacob(theta,i):
    rho_j=1
    beta_j=1
    alpha_j=1
    theta_min=np.array([-30, 0, 0])*math.pi/180 
    theta_max=np.array([30, 90, 60])*math.pi/180
    g= rho_j*(np.exp(-alpha_j*(theta_max[i]-theta))*pow(theta_max[i]-theta,-beta_j) + np.exp(-alpha_j*(theta-theta_min[i]))*pow(theta-theta_min[i],-beta_j))
    #g= rho_j*(np.exp(-alpha_j*(theta_max[i]-theta)) + np.exp(-alpha_j*(theta-theta_min[i])))
    return g

def hat_operator(rot):
    mat=np.array([[0,-rot[2],rot[1]],[rot[2],0,-rot[0]],[-rot[1],rot[0],0]])
    return mat

def rot_matrix(omega,theta):
    #print(hat_operator(omega))
    mat=np.identity(3)+sin(theta)*hat_operator(omega)+(1-cos(theta))*hat_operator(omega)@hat_operator(omega)
    #print(mat)
    return mat

def transformation_exp(xi,theta):
    trans=np.zeros((4,4))
    trans[:3,:3]=rot_matrix(xi[3:6],theta)
    trans[3,:4]=np.array([[0,0,0,1]])
    trans[:3,3]=((np.identity(3)-rot_matrix(xi[3:6],theta))@hat_operator(xi[3:6])@xi[0:3]).flatten()
    return trans

def adjoint(transformation):
    adj=np.zeros((6,6))
    adj[:3,:3]=transformation[:3,:3]
    adj[3:6,3:6]=transformation[:3,:3]
    adj[:3,3:6]=hat_operator(transformation[:3,3])@transformation[:3,:3]
    return adj

def main(data):
    global t
    global sub
    global sub1
    global sub2
    sub = data.position
    #sub=[0,0,0,0,0,0]
    print(sub)
    if t==0:
        sub1=np.array(sub)
        sub2=np.array(sub)
    else:
        sub1=sub2
        sub2=np.array(sub)
    
    #print(sub)
    pub = rospy.Publisher('/Geomagic1/force_feedback', OmniFeedback, queue_size=10)
    
    l_2=.147#.147
    l_1=.115
    #l_0=0.06#0.045
    K_m=1 # Force constant (Not sure about this as of now)
    xi1=np.array([[l_1],[0],[0],[0],[1],[0]])
    xi2=np.array([[0],[l_1],[0],[-1],[0],[0]])
    xi3=np.array([[0],[0],[l_2],[-1],[0],[0]])
    jacobian_matrix=np.zeros((6,3))
    jacobian_matrix[:6,0]=xi1.flatten()
    #print(adjoint(transformation_exp(xi1,np.pi/2)))
    T1=np.array([[cos(sub[0]),0,sin(sub[0]),l_1*sin(sub[0])],[0,1,0,0],[-sin(sub[0]),0,cos(sub[0]),-l_1*(1-cos(sub[0]))],[0,0,0,1]])
    T2=np.array([[1,0,0,0],[0,cos(sub[1]),sin(sub[1]),l_1*sin(sub[1])],[0,-sin(sub[1]),cos(sub[1]),-l_1*(1-cos(sub[1]))],[0,0,0,1]])
    T3=np.array([[1,0,0,0],[0,cos(sub[2]),sin(sub[2]),0],[0,-sin(sub[2]),cos(sub[2]),0],[0,0,0,1]])
    T12=T1@T2
    T123=T12@T3
    jacobian_matrix[:6,1]=(adjoint(T1)@xi2).flatten()
    #print(sub)
    jacobian_matrix[:6,2]=(adjoint(T12)@xi3).flatten()
    print((adjoint(T12)@xi3))
    grad_joint_limit=np.zeros((3,1))
    
    for i in range(3):
        if np.absolute(sub2[i]-sub1[i])>0:
            grad_joint_limit[i,0]=(gradient_jacob(sub2[i],i)-gradient_jacob(sub1[i],i))/(sub2[i]-sub1[i])
            if np.any(np.isnan(grad_joint_limit)):
                print(gradient_jacob(sub2[i],i)-gradient_jacob(sub1[i],i))
                #print(sub1)
            #print(np.linalg.norm(sub2-sub1))
            #print(sub2)
            #print(sub1)
            
    #print(grad_joint_limit)
    #print(jacobian_matrix)
    #print(grad_joint_limit.shape)
    #print(np.transpose(np.linalg.pinv(jacobian_matrix)).shape)
    #print(grad_joint_limit)

    #print(grad_joint_limit)
    
    Force=-1*K_m*np.transpose(np.linalg.pinv(jacobian_matrix))@grad_joint_limit
    Force_1=np.transpose(adjoint(T123))@Force
    
    Force=Force[:3]
    Force_1=Force_1[:3]
    force_msg = OmniFeedback()
    print(Force)
    if np.linalg.norm(grad_joint_limit)>100:
        Force=Force/np.linalg.norm(Force)
        Force_1=Force_1/np.linalg.norm(Force_1)
        #print("Post normalization",Force)
    else:
        Force=[0,0,0]
        Force_1=[0,0,0]
    #print(Force)
    
    force_msg.force.x  = Force[0]
    force_msg.force.y = Force[1]
    force_msg.force.z = Force[2]
    force_msg.lock =[0,0,0]
    pub.publish(force_msg)
    #force_msg.force.x  =0 
    #force_msg.force.y = 0
    #force_msg.force.z = 0
    #print(t)
    t=t+1
    


if __name__ == '__main__':
    try:
        Call()
    except rospy.ROnp.sinterruptException:
        pass
