#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

if __name__ == '__main__':
    rospy.init_node('swarm_test_high_level')

    cf_1 = crazyflie.Crazyflie("crazyflie1", "/crazyflie1")
    cf_2 = crazyflie.Crazyflie("crazyflie2", "/crazyflie2")

    # setup the cf's
    cf_1.setParam("commander/enHighLevel", 1)
    cf_1.setParam("stabilizer/estimator", 2) # Use EKF
    cf_1.setParam("stabilizer/controller", 2) # Use mellinger controller

    cf_2.setParam("commander/enHighLevel", 1)
    cf_2.setParam("stabilizer/estimator", 2) # Use EKF
    cf_2.setParam("stabilizer/controller", 2) # Use mellinger controller
    
    # reset initial positions
    cf_1.setParam("kalman/initialX", 0)
    cf_1.setParam("kalman/initialY", -0.5)
    cf_1.setParam("kalman/initialZ", 0)
    
    cf_2.setParam("kalman/initialX", 0)
    cf_2.setParam("kalman/initialY", 0.5)
    cf_2.setParam("kalman/initialZ", 0)
    
    # reset kalman
    cf_1.setParam("kalman/resetEstimation", 1)
    cf_2.setParam("kalman/resetEstimation", 1)
    
    cf_1.takeoff(targetHeight = 0.5, duration = 2.0)
    time.sleep(3.0)
    
    cf_2.takeoff(targetHeight = 0.5, duration = 2.0)
    time.sleep(3.0)
    
    cf_1.goTo(goal = [0.5, 0.0, 0.0], yaw=0, duration = 2.0, relative = True)
    cf_2.goTo(goal = [-0.5, 0.0, 0.0], yaw=0, duration = 2.0, relative = True)
    time.sleep(3.0)
    
    cf_1.goTo(goal = [0.0, 1, 0.0], yaw=0, duration = 2.0, relative = True)
    cf_2.goTo(goal = [0.0, -1, 0.0], yaw=0, duration = 2.0, relative = True)
    time.sleep(3.0)
    
    cf_2.goTo(goal = [1, 0.0, 0.0], yaw=0, duration = 2.0, relative = True)
    cf_1.goTo(goal = [-1, 0.0, 0.0], yaw=0, duration = 2.0, relative = True)
    time.sleep(3.0)

    cf_2.goTo(goal = [0.0, 1, 0.0], yaw=0, duration = 2.0, relative = True)
    cf_1.goTo(goal = [0.0, -1, 0.0], yaw=0, duration = 2.0, relative = True)
    time.sleep(3.0)

    cf_1.goTo(goal = [0.5, 0.0, 0.0], yaw=0, duration = 2.0, relative = True)
    cf_2.goTo(goal = [-0.5, 0.0, 0.0], yaw=0, duration = 2.0, relative = True)
    time.sleep(3.0)
  
    cf_1.land(targetHeight = 0.02, duration = 3.0)
    cf_2.land(targetHeight = 0.02, duration = 3.0)
    time.sleep(3.5)

    # traj1 = uav_trajectory.Trajectory()
    # traj1.loadcsv("takeoff.csv")

    # traj2 = uav_trajectory.Trajectory()
    # traj2.loadcsv("figure8.csv")

    # print(traj1.duration)

    # cf.uploadTrajectory(0, 0, traj1)
    # cf.uploadTrajectory(1, len(traj1.polynomials), traj2)

    # cf.startTrajectory(0, timescale=1.0)
    # time.sleep(traj1.duration * 2.0)

    # cf.startTrajectory(1, timescale=2.0)
    # time.sleep(traj2.duration * 2.0)

    # cf.startTrajectory(0, timescale=1.0, reverse=True)
    # time.sleep(traj1.duration * 1.0)

    cf_1.stop()
    cf_2.stop()
