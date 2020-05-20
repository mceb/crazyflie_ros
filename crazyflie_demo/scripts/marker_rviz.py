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


def callback(data, arg):
    publisher = arg
    
    rospy.loginfo(rospy.get_caller_id() + "I heard: %f, %f, %f", 
                  data.pose.position.x, data.pose.position.y, data.pose.position.z)

    marker = Marker(
                type=Marker.SPHERE,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(data.pose.position.x, data.pose.position.y, data.pose.position.z),
                          Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.35, 0.35, 0.7),
                header=Header(frame_id='/world'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
    publisher.publish(marker)
    
    
def republish():
    
    rospy.init_node('marker_republisher', anonymous=True)
    
    publisher = rospy.Publisher('vis_marker', Marker, queue_size=5)
    rospy.Subscriber('/crazyflie1/cmd_full_state', FullState, callback, (publisher))
    
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    republish()
