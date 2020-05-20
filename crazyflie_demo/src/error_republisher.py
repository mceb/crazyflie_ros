#!/usr/bin/env python

"""
Author: Mikulas Cebecauer
Date: 04/11/19

Describtion:
ROS republisher which computes the difference between supplied reference and tracked trajectory during execution onto error topic

Example:
https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/#17106

"""

import rospy
import tf

from nav_msgs.msg import Path
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point, Vector3
from std_msgs.msg import Header
from crazyflie_driver.msg import FullState

pose_sensor = Pose()
pose_reference = Pose()


def sensor_callback(data, verbose=False):
    
    if verbose: rospy.loginfo(rospy.get_caller_id() + "I heard: %f, %f, %f", 
                  data.pose.position.x, data.pose.position.y, data.poses.position.z)
    
    global pose_sensor
    print(pose_sensor)
    pose_sensor = data.pose
    
    
def reference_callback(data, arg, verbose=False):
    publisher_p = arg[0]
    publisher_o = arg[1]
    publisher_o_r = arg[2]
    publisher_o_s = arg[3]
    frame_id = arg[4]
    n_agent = arg[5]
    n_swarm = arg[6]
    
    if verbose: rospy.loginfo(rospy.get_caller_id() + "I heard: %f, %f, %f", 
                  data.pose.position.x, data.pose.position.y, data.pose.position.z)
    
    global pose_sensor
    pose_reference = data.pose
    
    pose_error = Pose()

    # compute pose difference (no real-time sampling guarantee for the samples used in computation)
    pose_error.position.x = pose_reference.position.x - pose_sensor.position.x
    pose_error.position.y = pose_reference.position.y - pose_sensor.position.y
    pose_error.position.z = pose_reference.position.z - pose_sensor.position.z
    
    pose_error.orientation.x = pose_reference.orientation.x - pose_sensor.orientation.x
    pose_error.orientation.y = pose_reference.orientation.y - pose_sensor.orientation.y
    pose_error.orientation.z = pose_reference.orientation.z - pose_sensor.orientation.z
    pose_error.orientation.w = pose_reference.orientation.w - pose_sensor.orientation.w
    
    publisher_p.publish(pose_error)
    
    euler_error = Vector3()
    
    # compute difference in euler angles (no real-time sampling guarantee for the samples used in computation)
    (r_s, p_s, y_s) = tf.transformations.euler_from_quaternion([pose_sensor.orientation.x, pose_sensor.orientation.y,
                                                                pose_sensor.orientation.z, pose_sensor.orientation.w])
    (r_r, p_r, y_r) = tf.transformations.euler_from_quaternion([pose_reference.orientation.x, pose_reference.orientation.y,
    
    pose_reference.orientation.z, pose_reference.orientation.w])
    euler_error.x = (r_r - r_s) * 57.2958 #roll
    euler_error.y = (p_r - p_s) * 57.2958 #pitch
    euler_error.z = (y_r - y_s) * 57.2958 #yaw
    
    publisher_o.publish(euler_error)
    
    euler_r = Vector3()
    euler_s = Vector3()

    # republish the raw orientations (ref, sensor) in euler deg
    euler_r.x = r_r * 57.2958 #roll
    euler_r.y = p_r * 57.2958 #pitch
    euler_r.z = y_r * 57.2958 #yaw
    publisher_o_r.publish(euler_r)
    euler_s.x = r_s * 57.2958 #roll
    euler_s.y = p_s * 57.2958 #pitch
    euler_s.z = y_s * 57.2958 #yaw
    publisher_o_s.publish(euler_s)
    
    
def republish(ns, sensor_topic, reference_topic, frame_id, n_agent, n_swarm):
    rospy.loginfo("Starting Republisher")
    publisher_p = rospy.Publisher(ns + 'p_error', Pose, queue_size=5)
    publisher_o = rospy.Publisher(ns + 'o_error', Vector3, queue_size=5)
    publisher_o_r = rospy.Publisher(ns + 'orientation_r', Vector3, queue_size=5)
    publisher_o_s = rospy.Publisher(ns + 'orientation_s', Vector3, queue_size=5)
    #rospy.Subscriber('/' + sensor_topic + ns + 'pose', PoseStamped, sensor_callback)
    #rospy.Subscriber(reference_topic, FullState, reference_callback, (publisher_p, publisher_o, publisher_o_r, publisher_o_s, frame_id, n_agent, n_swarm))
    
    rospy.Subscriber(sensor_topic, PoseStamped, sensor_callback)
    rospy.Subscriber(reference_topic, FullState, reference_callback, (publisher_p, publisher_o, publisher_o_r, publisher_o_s, frame_id, n_agent, n_swarm))
    
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('error_republisher', anonymous=True)
    
    n_agent = rospy.get_param('~n_agent')
    n_swarm = rospy.get_param('~n_swarm')
    sensor_topic = rospy.get_param('~sensor_topic')
    reference_topic = rospy.get_param('~reference_topic')
    frame_id = rospy.get_param('~frame_id')
    
    ns = rospy.get_namespace()
    
    try:
        republish(ns, sensor_topic, reference_topic, frame_id, n_agent, n_swarm)
    except:
        rospy.loginfo("Failed")
        pass
