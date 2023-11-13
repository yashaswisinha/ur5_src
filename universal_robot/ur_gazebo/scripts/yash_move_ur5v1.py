#!/usr/bin/env python3
#!/usr/bin/python3
#
# Send joint values to UR5 using messages
#
import time
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from gazebo_msgs.msg import LinkStates
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float32MultiArray
import rospy
import actionlib
import sys, select, termios, tty ``
from sensor_msgs.msg import JointState
import tf
#reload(sys) 
#sys.setdefaultencoding("UTF-8")
#from ur_kinematics import Kinematics
from math import sin,cos
import numpy as np


print("Mai gadha hun")

def Call():
    
    rospy.init_node('Joint_controller', anonymous=True)
    # setup Phantom Omni joint states
    #rospy.Subscriber('/Geomagic1/joint_states', JointState, main, queue_size=1)
    rospy.Subscriber('/gazebo/link_states', LinkStates, main, queue_size=1)
    #rospy.Subscriber("arm_controller/state",JointTrajectoryControllerState, main, queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
# callback_State() returns 'current_state' global variable
def ur5(state):
    sub = state.position
    print(sub)

def transformation(a,d,theta,alpha):
    T=np.array([[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)],[sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta)],[0,sin(alpha),cos(alpha),d],[0,0,0,1]])
    return T

def main(data):
#def main():
    #bias_robot1=np.array([0.053890,-1.040424,1])
    tracked_object=np.array([-0.035478, 0.112842, 0.678375])
    #tracked_object_wrt_robot_base=tracked_object
    global pose
    pub = rospy.Publisher('/robot1/eff_joint_traj_controller/command',JointTrajectory,queue_size=1)
    sub = [-np.pi/2,-1,1.1,-np.pi/2,0,0]
    #print(data)
    #print(sub)
    #Getting the joint values so the end effector aligns to the object
    gazebo_base_transformation=np.array([[-1,0,0,-0.008724],[0,-1,0,-0.946393],[0,0,1,0.66],[0,0,0,1]])
    link_index = data.name.index("robot1::wrist_3_link")
    ee=np.array([data.pose[link_index].position.x,data.pose[link_index].position.y,data.pose[link_index].position.z])
    ee_orientation=np.array([data.pose[link_index].orientation.x,data.pose[link_index].orientation.y,data.pose[link_index].orientation.z])
    #print(ee)
    T1=gazebo_base_transformation@transformation(0,0.089459,sub[0],np.pi/2) #DH parameters
    T2=T1@transformation(-0.425,0,sub[1],0)
    T3=T2@transformation(-0.392,0,sub[2],0)
    T4=T3@transformation(0,0.109,sub[3],np.pi/2)
    T5=T4@transformation(0,0.094,sub[4],-np.pi/2)
    T6=T5@transformation(0,0.082,sub[5],0)

    print("T1",T1[:3,3])
    print("T2",T2[:3,3])
    print("T3",T3[:3,3])
    print("T4",T4[:3,3])
    print("T5",T5[:3,3])
    print("T6",T6[:3,3])
    trans=T6[:3,3]
    dirn_vector=tracked_object-trans
    vec_ee_frame=T6[:3,:3]@np.transpose(dirn_vector)
    ang2=np.arctan(dirn_vector[0]/dirn_vector[1])
    sub[0]=sub[0]-ang2/2
    #z_axis=
    #ang_pitch=np.arccos(np.dot(z_axis_ee))
    #if np.linalg.norm(dirn_vector-ee_orientation)>0.1:
        #ang2=np.arctan(dirn_vector[0]/dirn_vector[1])
        #ang1=np.arccos(np.sqrt(dirn_vector[0]**2+dirn_vector[1]**2)/np.linalg.norm(dirn_vector))
        #sub[4]=sub[4]+ang2
        #sub[3]=sub[3]+ang1
    #ang1=np.arcsin(dirn_vector[1]/np.linalg.norm(dirn_vector))
    #ang2=np.arcsin(dirn_vector[0]/cos(sub[3])*np.linalg.norm(dirn_vector))
    #
    
    #print(ang1)
    #print("ang2",ang2)
    #print(T6[:3,3])
    #print(T1[:3,3])
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    Q0=np.array([sub[0], sub[1],sub[2], sub[3], sub[4], sub[5] ])
    point = JointTrajectoryPoint()
    point.positions = Q0
    point.time_from_start = rospy.Duration(5) # Adjust the duration as needed

    joint_trajectory.points.append(point)
    
    
    #pts = JointTrajectoryPoint()


    #user_value = -0.15
    rate = rospy.Rate(10) # 10hz
    pub.publish(joint_trajectory)
    rate.sleep()
  

'''
    pts.positions = Q0
    pts.time_from_start = rospy.Duration(0.1)
  
    traj.points = []
    traj.points.append(pts)
    pub.publish(traj)'''
    
    #rate.sleep()


if __name__ == '__main__':
    try:
        Call()
        #Phantom
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")

'''
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_ur5(joint_angles):
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('ur5_joint_mover', anonymous=True)

    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    point = JointTrajectoryPoint()
    point.positions = joint_angles
    point.time_from_start = rospy.Duration(5) # Adjust the duration as needed

    joint_trajectory.points.append(point)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print("Hello?")
        pub.publish(joint_trajectory)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Example joint angles (in radians)
        angles = [0.0, -1.57, 1.0, -1.0, 1.0, 0.0]
        move_ur5(angles)
    except rospy.ROSInterruptException:
        pass

'''