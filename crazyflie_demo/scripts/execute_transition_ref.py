#!/usr/bin/env python

"""
Author: Mikulas Cebecauer
Date: 05/11/19

Describtion:
ROS publisher which executes precomputed trajectory & transition

Example:
python execure_transitions.py -l 1234.csv 2345.csv 3456.npy 4567.npy

"""

import argparse
import rospy
import logging
import tf_conversions
from crazyflie_driver.msg import FullState
import geometry_msgs
import uav_trajectory
import numpy as np


class Transition:
  def __init__(self, h=0.01):
    self.output = None
    self.duration = None
    self.h = h

  def loadnpy(self, file_name):
    self.output = np.load(file_name) 
    self.duration = len(self.output[:,0,0])*self.h
    
def loader():
    parser = argparse.ArgumentParser()
    parser.add_argument("-l","--list", nargs="+", 
                        help="list CSV files containing ordered trajectories/transitions", required=True) 
    
    for _, arg_list in parser.parse_args()._get_kwargs():
        if arg_list is not None:
            
            # unpack each of the trajectory/transition
            plan = np.zeros((len(arg_list)), dtype=object)
            for idx, file_name in enumerate(arg_list):
                
                if 'trajectory' in file_name:
                    plan[idx] = uav_trajectory.Trajectory()
                    try:
                        plan[idx].loadcsv(file_name)
                    except:
                        return False
                    
                if 'transition' in file_name:
                    plan[idx] = Transition()
                    try:
                        plan[idx].loadnpy(file_name)
                    except:
                        return False
                    
            return plan
        
        else:
            return False
    # should never get here


def check_continuity(plan, verbose=False, h=0.01):
    # tolerance of each state to satisfy continuity condition (make non-zero)
    tolerance = np.array([0.05, 0.05, 0.05, 
                          0.01, 0.01, 0.01, 
                          0.001, 0.001, 0.001, 
                          0.01, 
                          0.01, 0.01, 0.01])
    
    for idx in range(len(plan)-1):
        # final state
        if isinstance(plan[idx], uav_trajectory.Trajectory):
            e = plan[idx].eval(plan[idx].duration - h)
            xf = np.array([e.pos[0], e.pos[1], e.pos[2],
                           e.vel[0], e.vel[1], e.vel[2],
                           e.acc[0], e.acc[1], e.acc[2],
                           e.yaw, 
                           e.omega[0], e.omega[1], e.omega[2]])
            if verbose:
                np.set_printoptions(suppress=True)
                print('last:', xf)
        
        if isinstance(plan[idx], Transition):
            xf = np.array([plan[idx].output[-1,0,0],
                           plan[idx].output[-1,0,1],
                           plan[idx].output[-1,0,2],
                           plan[idx].output[-1,0,3],
                           plan[idx].output[-1,0,4],
                           plan[idx].output[-1,0,5],
                           plan[idx].output[-1,0,6],
                           plan[idx].output[-1,0,7],
                           plan[idx].output[-1,0,8],
                           0,0,0,0]) # yaw & omega not computed
            if verbose:
                np.set_printoptions(suppress=True)
                print('last:', xf)
            
        # initial state 
        if isinstance(plan[idx+1], uav_trajectory.Trajectory):
            e = plan[idx+1].eval(0) 
            xi = np.array([e.pos[0], e.pos[0], e.pos[0],
                           e.vel[0], e.vel[0], e.vel[0],
                           e.acc[0], e.acc[0], e.acc[0],
                           e.yaw, 
                           e.omega[0], e.omega[0], e.omega[0]])
            if verbose:
                np.set_printoptions(suppress=True)
                print('first:', xi)
            
        if isinstance(plan[idx+1], Transition):
            xi = np.array([plan[idx+1].output[0,0,0],
                          plan[idx+1].output[0,0,1],
                          plan[idx+1].output[0,0,2],
                          plan[idx+1].output[0,0,3],
                          plan[idx+1].output[0,0,4],
                          plan[idx+1].output[0,0,5],
                          plan[idx+1].output[0,0,6],
                          plan[idx+1].output[0,0,7],
                          plan[idx+1].output[0,0,8],
                          0,0,0,0]) # yaw & omega not computed
            if verbose:
                np.set_printoptions(suppress=True)
                print('first:', xi)
            
        # all states must be within tolerance (continuity condition)
        for i in range(len(xf)):
            if float(xf[i] - xi[i]) >= tolerance[i]:
                return False
            
    return True
    

def execute_trajectory(pub, rate, msg, traj):
    start_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        t = (msg.header.stamp - start_time).to_sec()
        print(t)
        if t > traj.duration:
            break
            # t = traj.duration

        e = traj.eval(t)

        msg.pose.position.x    = e.pos[0]
        msg.pose.position.y    = e.pos[1]
        msg.pose.position.z    = e.pos[2]
        msg.twist.linear.x     = e.vel[0]
        msg.twist.linear.y     = e.vel[1]
        msg.twist.linear.z     = e.vel[2]
        msg.acc.x              = e.acc[0]
        msg.acc.y              = e.acc[1]
        msg.acc.z              = e.acc[2]
        msg.pose.orientation   = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, e.yaw))
        msg.twist.angular.x    = e.omega[0]
        msg.twist.angular.y    = e.omega[1]
        msg.twist.angular.z    = e.omega[2]

        pub.publish(msg)
        rate.sleep()
    
    return


def execute_transition(pub, pub_ref, rate, msg, trans, ref):
    start_time = rospy.Time.now()
    
    count = 0
    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        t = (msg.header.stamp - start_time).to_sec()
        print(t)
        if t > trans.duration or count >= trans.duration*100:
            break
        
        # this executes only transition of agent 0!
        msg.pose.position.x    = trans.output[count,0,0]
        msg.pose.position.y    = trans.output[count,0,1]
        msg.pose.position.z    = trans.output[count,0,2]
        msg.twist.linear.x     = trans.output[count,0,3]
        msg.twist.linear.y     = trans.output[count,0,4]
        msg.twist.linear.z     = trans.output[count,0,5]
        msg.acc.x              = trans.output[count,0,6]
        msg.acc.y              = trans.output[count,0,7]
        msg.acc.z              = trans.output[count,0,8]
        msg.pose.orientation   = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
        msg.twist.angular.x    = 0
        msg.twist.angular.y    = 0
        msg.twist.angular.z    = 0

        pub.publish(msg)
        count += 1 
        
        ref.header.seq += 1
        ref.header.stamp = rospy.Time.now()
        pub_ref.publish(ref)
        
        rate.sleep()
    
    return

    
if __name__ == '__main__':
    
    plan = loader()

    # check that the plan is realisible
    if check_continuity(plan):
        
        rospy.init_node('full_state')
        
        rate = rospy.Rate(100)
        
        msg = FullState()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/world"

        ref = FullState()
        ref.header.seq = 0
        ref.header.stamp = rospy.Time.now()
        ref.header.frame_id = "/world"
        ref.twist.linear.x     = 0
        ref.twist.linear.y     = 0
        ref.twist.linear.z     = 0
        ref.acc.x              = 0
        ref.acc.y              = 0
        ref.acc.z              = 0
        ref.pose.orientation   = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
        ref.twist.angular.x    = 0
        ref.twist.angular.y    = 0
        ref.twist.angular.z    = 0
        
        pub = rospy.Publisher("/crazyflie1/cmd_full_state", FullState, queue_size=1)
        pub_ref = rospy.Publisher("/crazyflie1/ref_state", FullState, queue_size=1)
        
        # x, y, z (ass supplied in MPC simulator), assuming yaw = 0 const.
        ref_points = [[-0.5, 0.0, 1.0],
                      [-0.5, 1.0, 1.0],
                      [0.5, 1.0, 1.0],
                      [-0.5, 0.0, 1.0],
                      [-0.5, 0.0, 0.0]]
        
        print("Start Execution \r\n")
        idx = 0
        for sub_plan in plan:
            print("Executing:" +  str(sub_plan) + "\r\n")
            
            ref.pose.position.x = ref_points[int(idx)][0]
            ref.pose.position.y = ref_points[int(idx)][1]
            ref.pose.position.z = ref_points[int(idx)][2]
            
            if isinstance(sub_plan, uav_trajectory.Trajectory):
                execute_trajectory(pub, rate, msg, sub_plan)
            
            if isinstance(sub_plan, Transition):
                execute_transition(pub, pub_ref, rate, msg, sub_plan, ref)
            
            idx +=1
            
        print("Finished Execution \r\n")
    
    else:
        print("Continuity Not Satisfied \r\n")
    
