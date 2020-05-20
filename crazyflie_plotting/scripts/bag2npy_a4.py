
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
save_name = '/data-agent4.npy'

np_dtype_list = np.array([[],[],[],[],[],[],[]])

def callback(msg):
    global np_dtype_list
    #pose stamped
    #np_dtype_list = np.append(np_dtype_list, [[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z],
    #                                          [msg.pose.orientation.x], [msg.pose.orientation.y], [msg.pose.orientation.z],
    #                                          [msg.pose.orientation.w]] , axis=1)
    
    #pose
    np_dtype_list = np.append(np_dtype_list, [[msg.position.x], [msg.position.y], [msg.position.z],
                                              [msg.orientation.x], [msg.orientation.y], [msg.orientation.z],
                                              [msg.orientation.w]] , axis=1)
    
    
    global save_path, save_name
    today = datetime.now()
    if not os.path.exists(save_path + today.strftime('%Y%m%d')):
        os.mkdir(save_path + today.strftime('%Y%m%d'))
    
    print('save')
    np.save(save_path + today.strftime('%Y%m%d') + save_name, np_dtype_list)

    pass


if __name__=='__main__':
    
    rospy.init_node('bag2npy_a4')
    
    # for saving pose
    #subscriber = rospy.Subscriber('/vrpn_client_node/crazyflie4/pose/', PoseStamped, callback)
    
    # for saving error
    subscriber = rospy.Subscriber('/crazyflie4/p_error/', Pose, callback)
    rospy.spin()

    print('success')
