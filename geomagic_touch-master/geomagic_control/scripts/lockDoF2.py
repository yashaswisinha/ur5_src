#!/usr/bin/env python
# THIS SCRIPT WAS JUST TO TEST STUFF AND SHOULD NOT  BE USED ANY MORE
import rospy
import math
from sensor_msgs.msg import JointState
from geomagic_control.msg import OmniFeedback
import tf
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float32MultiArray
import time

def PS(origin,position=[0,0,0],orientation=[0,0,0,1]):
    """
        Creates a PoseStamped()
        
        :param origin: frame id
        :type: origin: str
        :param position: position (default [0,0,0])
        :type: position: list
        :param oriention: orientation (default [0,0,0,1])
        :type: orientation: list
        :return: the created PoseStamped()
        :rtype: PoseStamped
    """
    h=Header()
    h.frame_id=origin
    h.stamp=rospy.Time().now()
    
    p=Pose()
    if type(position) == Point:
        p.position = position
    else:
        p.position=Point(*position)
    if type(orientation) == Quaternion:
        p.orientation = orientation 
    elif len(orientation) == 4:
        p.orientation=Quaternion(*orientation)
    elif len(orientation) == 3:
        p.orientation = Quaternion(*tf.transformations.quaternion_from_euler(*orientation))
    else:
        p.orientation = Quaternion(0,0,0,1)
    return PoseStamped(h,p)
class GeomagicInterface:
    def __init__(self):
#         self.listener = tf.TransformListener()
        self.of = OmniFeedback()
        self.of.lock = [False for i in xrange(3)]
        self.pub = rospy.Publisher("Geomagic1/force_feedback",OmniFeedback,queue_size=1)
        # self.sub = rospy.Subscriber("Geomagic/end_effector_pose",JointState,self.callback, queue_size=1)
        self.lock = 0.0
        self.sub = rospy.Subscriber("Geomagic1/joint_states",JointState,self.callback, queue_size=1)
        self.sub1 = rospy.Subscriber("robot2/arm_controller/state",JointTrajectoryControllerState,self.call, queue_size=1)
    
    def call(self,unr):
        global pan 
        global lift
	global elbow
        pan = unr.actual.positions[0]
        lift = unr.actual.positions[1]
	elbow = unr.actual.positions[2]
        #rospy.loginfo('force xu:{}'.format(pan))
    def callback(self,js):
        #x = 10.0
        global waist
        global lift1
	global elbow1
        waist = js.position[0]+3.14
        lift1 = -js.position[1]
	elbow1 = -(js.position[2]-1.31)
        #pan = js.actual.positions[0]
        self.of.force.x = (pan-waist)*4
        self.of.force.y =  (lift1-lift)*5
	self.of.force.z =  (elbow1-elbow)*5
     #   self.of.force.y =0 
     #   self.of.force.z =0#0.04 *(s)elf.lock - x) - 0.001 * js.velocity[0];
        #self.of.position.x = js.position[0]
        #self.of.position.y = js.position[1]
        #self.of.position.z = js.position[2]
        self.of.lock = [False,False,False]
        self.pub.publish(self.of) 
        rospy.loginfo('force x:{}, force xo:{},force xu:{}'.format(self.of.force.x,waist,pan))
        rospy.loginfo('force y:{}, force yo:{},force yu:{}'.format(self.of.force.y,lift,lift1))
	rospy.loginfo('force z:{}, force zo:{},force zu:{}'.format(self.of.force.z,elbow,elbow1))
        
#     def start(self):
#         while not rospy.is_shutdown():
#             x = self.getPose("tip", "base").pose.position.x
#             print self.of.force.x
#             self.pub.publish(self.of)
#             rospy.sleep(0.1)

#     def getPose(self,target,source,timeout=1):
#         now = rospy.Time.now()
#         end = rospy.Time.now()+rospy.Duration(timeout)
#         while not rospy.is_shutdown() and now < end:
#             now = rospy.Time.now()
#             try:
#                 (trans,rot) = self.listener.lookupTransform(source,target, rospy.Time(0))
#                 return PS(source,trans,rot)
#             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                 rospy.sleep(0.01)
#                 continue
#         raise Exception,"Transform %s -> %s never appeared"%(target,source)
#         return None
        
if __name__ == '__main__':
    
    rospy.init_node("geomagic1_touch_dof_locker")
    gi = GeomagicInterface()
    
    
    


    rospy.spin()
