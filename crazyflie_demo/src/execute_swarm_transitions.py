#!/usr/bin/env python

"""
Author: Mikulas Cebecauer
Date: 10/10/19

Describtion:
ROS publisher which executes precomputed agent transition form swarm transitions 

"""

import rospy
import tf_conversions
from crazyflie_driver.msg import FullState
from std_msgs.msg import Empty
import geometry_msgs
import numpy as np


class Transition:
  def __init__(self, h=0.01):
    self.output = None
    self.duration = None
    self.h = h
    
  def loadnpy(self, file_name):
    self.output = np.load(file_name) 
    self.duration = len(self.output[:,0,0])*self.h
    
    
def loader(trans_list, path):
    
    if trans_list is not None:
        
        # unpack each of the trajectory/transition
        plan = np.zeros((len(trans_list)), dtype=object)
        for idx, file_name in enumerate(trans_list):
            
            if '.npy' in file_name:
                plan[idx] = Transition()
                
                try:
                    plan[idx].loadnpy(path + str(file_name))
                except:
                    rospy.logwarn('No file loaded')
                    return False
                
        return plan
    
    else:
        rospy.logwarn('No file loaded')
        return False
    # should never get here


def check_continuity(plan, n_agent, verbose=False, h=0.01):
    # tolerance of each state to satisfy continuity condition (make non-zero)
    tolerance = np.array([0.1, 0.1, 0.1, 
                          0.1, 0.1, 0.1, 
                          0.1, 0.1, 0.1, 
                          0.1, 
                          0.1, 0.1, 0.1])


    for idx in range(len(plan)-1):
        if isinstance(plan[idx], Transition):
            xf = np.array([plan[idx].output[-1,n_agent,0],
                            plan[idx].output[-1,n_agent,1],
                            plan[idx].output[-1,n_agent,2],
                            plan[idx].output[-1,n_agent,3],
                            plan[idx].output[-1,n_agent,4],
                            plan[idx].output[-1,n_agent,5],
                            plan[idx].output[-1,n_agent,6],
                            plan[idx].output[-1,n_agent,7],
                            plan[idx].output[-1,n_agent,8],
                            0,0,0,0]) # yaw & omega not computed
            if verbose:
                np.set_printoptions(suppress=True)
                print('last:', xf)
            
        if isinstance(plan[idx+1], Transition):
            xi = np.array([plan[idx+1].output[0,n_agent,0],
                            plan[idx+1].output[0,n_agent,1],
                            plan[idx+1].output[0,n_agent,2],
                            plan[idx+1].output[0,n_agent,3],
                            plan[idx+1].output[0,n_agent,4],
                            plan[idx+1].output[0,n_agent,5],
                            plan[idx+1].output[0,n_agent,6],
                            plan[idx+1].output[0,n_agent,7],
                            plan[idx+1].output[0,n_agent,8],
                            0,0,0,0]) # yaw & omega not computed
            if verbose:
                np.set_printoptions(suppress=True)
                print('first:', xi)
            
        # all states must be within tolerance (continuity condition)
        for i in range(len(xf)):
            if float(xf[i] - xi[i]) >= tolerance[i]:
                return False
            
    return True


def execute_transition(pub, rate, msg, trans, n_agent, verbose=False):
    start_time = rospy.Time.now()
    
    count = 0
    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        t = (msg.header.stamp - start_time).to_sec()
        
        if verbose: rospy.loginfo(t)
        if t > trans.duration or count >= (trans.duration-1)*100:
            break
        
        # this executes only transition of n_agent!
        msg.pose.position.x    = trans.output[count,n_agent,0]
        msg.pose.position.y    = trans.output[count,n_agent,1]
        msg.pose.position.z    = trans.output[count,n_agent,2]
        msg.twist.linear.x     = trans.output[count,n_agent,3]
        msg.twist.linear.y     = trans.output[count,n_agent,4]
        msg.twist.linear.z     = trans.output[count,n_agent,5]
        msg.acc.x              = trans.output[count,n_agent,6]
        msg.acc.y              = trans.output[count,n_agent,7]
        msg.acc.z              = trans.output[count,n_agent,8]
        msg.pose.orientation   = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
        msg.twist.angular.x    = 0
        msg.twist.angular.y    = 0
        msg.twist.angular.z    = 0

        pub.publish(msg)
        count += 1 
        rate.sleep()
    
    return

    
if __name__ == '__main__':
    
    rospy.init_node('full_state_swarm')
    
    n_agent = rospy.get_param('~n_agent')
    pub_topic = rospy.get_param('~pub_topic')
    frame_id = rospy.get_param('~frame_id')
    path = rospy.get_param('~path')
    trans_list = rospy.get_param('~transitions_list')
    trans_list = trans_list.replace("'","").split(',')
    
    plan = loader(trans_list, path)
    
    ns = rospy.get_namespace()
    
    # check that the plan is realisible
    if check_continuity(plan, n_agent):
        
        rate = rospy.Rate(100)
        
        msg = FullState()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        
        pub = rospy.Publisher(ns + pub_topic, FullState, queue_size=1)
        stop_pub = rospy.Publisher(ns + 'cmd_stop', Empty, queue_size=1)
        
        rospy.loginfo('Start Execution')
        for sub_plan in plan:
            rospy.loginfo('Executing:' +  str(sub_plan))
            
            if isinstance(sub_plan, Transition):
                execute_transition(pub, rate, msg, sub_plan, n_agent)
        
        stop_pub.publish(Empty())
        
        rospy.loginfo('Finished Execution')
    
    else:
        rospy.loginfo('Continuity Not Satisfied')
    
