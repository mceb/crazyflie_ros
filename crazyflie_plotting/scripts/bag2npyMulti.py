
"""
Author: Mikulas Cebecauer
Date: 04/11/19

Describtion:
ROS pose bag2npy

"""

import sys
import os
from datetime import datetime
#os.system('source /opt/ros/kinetic/setup.bash')

import numpy as np
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


save_path = '/home/mikulas/crazyflie_ws/src/crazyflie_ros/crazyflie_plotting/data/'
save_name_1 = '/data-agent1.npy'
save_name_2 = '/data-agent2.npy'
save_name_3 = '/data-agent3.npy'
save_name_4 = '/data-agent4.npy'
save_name_5 = '/data-agent5.npy'

np_dtype_list = np.array([[],[],[],[],[],[],[]])

def callback1(msg):
    global np_dtype_list
    #pose stamped
    np_dtype_list = np.append(np_dtype_list, [[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z],
                                              [msg.pose.orientation.x], [msg.pose.orientation.y], [msg.pose.orientation.z],
                                              [msg.pose.orientation.w]] , axis=1)
    
    global save_path, save_name_1
    today = datetime.now()
    if not os.path.exists(save_path + today.strftime('%Y%m%d')):
        os.mkdir(save_path + today.strftime('%Y%m%d'))
    
    np.save(save_path + today.strftime('%Y%m%d') + save_name_1, np_dtype_list)

    pass


def callback2(msg):
    global np_dtype_list
    #pose stamped
    np_dtype_list = np.append(np_dtype_list, [[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z],
                                              [msg.pose.orientation.x], [msg.pose.orientation.y], [msg.pose.orientation.z],
                                              [msg.pose.orientation.w]] , axis=1)
    
    global save_path, save_name_2
    today = datetime.now()
    if not os.path.exists(save_path + today.strftime('%Y%m%d')):
        os.mkdir(save_path + today.strftime('%Y%m%d'))
    
    np.save(save_path + today.strftime('%Y%m%d') + save_name_2, np_dtype_list)

    pass


def callback3(msg):
    global np_dtype_list
    #pose stamped
    np_dtype_list = np.append(np_dtype_list, [[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z],
                                              [msg.pose.orientation.x], [msg.pose.orientation.y], [msg.pose.orientation.z],
                                              [msg.pose.orientation.w]] , axis=1)
    
    global save_path, save_name_3
    today = datetime.now()
    if not os.path.exists(save_path + today.strftime('%Y%m%d')):
        os.mkdir(save_path + today.strftime('%Y%m%d'))
    
    np.save(save_path + today.strftime('%Y%m%d') + save_name_3, np_dtype_list)

    pass


def callback4(msg):
    global np_dtype_list
    #pose stamped
    np_dtype_list = np.append(np_dtype_list, [[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z],
                                              [msg.pose.orientation.x], [msg.pose.orientation.y], [msg.pose.orientation.z],
                                              [msg.pose.orientation.w]] , axis=1)

    global save_path, save_name_4
    today = datetime.now()
    if not os.path.exists(save_path + today.strftime('%Y%m%d')):
        os.mkdir(save_path + today.strftime('%Y%m%d'))
    
    np.save(save_path + today.strftime('%Y%m%d') + save_name_4, np_dtype_list)

    pass


def callback5(msg):
    global np_dtype_list
    #pose stamped
    np_dtype_list = np.append(np_dtype_list, [[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z],
                                              [msg.pose.orientation.x], [msg.pose.orientation.y], [msg.pose.orientation.z],
                                              [msg.pose.orientation.w]] , axis=1)    
    
    global save_path, save_name_5
    today = datetime.now()
    if not os.path.exists(save_path + today.strftime('%Y%m%d')):
        os.mkdir(save_path + today.strftime('%Y%m%d'))
    
    np.save(save_path + today.strftime('%Y%m%d') + save_name_5, np_dtype_list)

    pass



if __name__=='__main__':
    
    rospy.init_node('bag2npy')
    
    # for saving pose
    subscriber = rospy.Subscriber('/vrpn_client_node/crazyflie1/pose/', PoseStamped, callback1)
    subscriber = rospy.Subscriber('/vrpn_client_node/crazyflie2/pose/', PoseStamped, callback2)
    subscriber = rospy.Subscriber('/vrpn_client_node/crazyflie3/pose/', PoseStamped, callback3)
    subscriber = rospy.Subscriber('/vrpn_client_node/crazyflie4/pose/', PoseStamped, callback4)
    subscriber = rospy.Subscriber('/vrpn_client_node/crazyflie5/pose/', PoseStamped, callback5)
    
    rospy.spin()

    print('success')
