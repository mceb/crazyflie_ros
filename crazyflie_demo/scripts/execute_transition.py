#!/usr/bin/env python

"""
Author: Mikulas Cebecauer
Date: 08/10/19

Describtion:
ROS publisher which executes precomputed swarm transition

"""

import argparse
import rospy
import tf_conversions
from crazyflie_driver.msg import FullState
import geometry_msgs
import uav_trajectory


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory", type=str, help="CSV file containing trajectory")
    parser.add_argument('-l','--list', action='append', help='<Required> Set flag', required=True)
    args = parser.parse_args()

    rospy.init_node('full_state')

    traj = uav_trajectory.Trajectory()
    traj.loadcsv(args.trajectory)

    rate = rospy.Rate(100)

    msg = FullState()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/world"

    pub = rospy.Publisher("/crazyflie2/cmd_full_state", FullState, queue_size=1)
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
