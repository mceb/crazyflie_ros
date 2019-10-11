#!/usr/bin/env python

"""
Author: Mikulas Cebecauer
Date: 10/10/19

Describtion:
ROS republisher which is listens to custom message & publishes pose in standard message for rviz

Resources:
https://answers.ros.org/question/231492/passing-arguments-to-callback-in-python/
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

"""

import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from crazyflie_driver.msg import FullState


def callback(data, arg, verbose=False):
    publisher = arg[0]
    frame_id = arg[1]
    n_agent = arg[2]
    n_swarm = arg[3]
    
    if verbose: rospy.loginfo(rospy.get_caller_id() + "I heard: %f, %f, %f", 
                  data.pose.position.x, data.pose.position.y, data.pose.position.z)
    
    marker = Marker(
                type=Marker.SPHERE,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(data.pose.position.x, data.pose.position.y, data.pose.position.z),
                          Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.35, 0.35, 0.7),
                header=Header(frame_id=frame_id),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
    publisher.publish(marker)
    
    
def republish(ns, sub_topic, frame_id, n_agent, n_swarm):
    publisher = rospy.Publisher(ns + 'vis_marker', Marker, queue_size=5)
    rospy.Subscriber(sub_topic, FullState, callback, (publisher, frame_id, n_agent, n_swarm))
    
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('marker_republisher', anonymous=True)
    
    n_agent = rospy.get_param('~n_agent')
    n_swarm = rospy.get_param('~n_swarm')
    sub_topic = rospy.get_param('~sub_topic')
    frame_id = rospy.get_param('~frame_id')
    
    ns = rospy.get_namespace()
    
    try:
        republish(ns, sub_topic, frame_id, n_agent, n_swarm)
    except:
        pass
