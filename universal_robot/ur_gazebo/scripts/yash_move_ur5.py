#!/usr/bin/env python3
#!/usr/bin/python3
#
# Send joint values to UR5 using messages
#
import time
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float32MultiArray
import rospy
import actionlib
import sys, select, termios, tty 
from sensor_msgs.msg import JointState
import tf
#reload(sys) 
#sys.setdefaultencoding("UTF-8")
#from ur_kinematics import Kinematics
import math
import numpy as np




def Call():
    
    rospy.init_node('Joint_controller', anonymous=True)
    # setup Phantom Omni joint states
    rospy.Subscriber('/Geomagic1/joint_states', JointState, main, queue_size=1)
    #rospy.Subscriber("arm_controller/state",JointTrajectoryControllerState, main, queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
# callback_State() returns 'current_state' global variable
def Phantom(state):
    global sub
    sub = state.position
    print(sub)

#def ur_get_status():
#    current_data = rospy.wait_for_message("arm_controller/state", JointTrajectoryControllerState)
#    return current_data
#Q0 = [-0.12694727059672406, -1.331667696607827, 2.391941365528808, -1.1109140138393911, 1.545242764007165, 0.13237981553654432]

def main(data):
   	
    global pose
    pub = rospy.Publisher('/robot1/eff_joint_traj_controller/command',JointTrajectory,queue_size=1)
    sub = data.position
    #print(sub)
    print(sub)
 
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'camera_joint']
    Q0=np.array([sub[0], -sub[1], -sub[2]+1.8, 0, 0, 0, 1 ])
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