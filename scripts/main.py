#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
from helper import detector, CONTROLLER, avoidance, FLIGHT_ICUAS
from broadcaster import handle_pose


vel = 1.2
angular_vel = 0.4




if __name__ == '__main__' :
    rospy.init_node('icuas_mission', anonymous = True)

    rospy.Subscriber('/red/pose',
                     PoseStamped,
                     handle_pose)
    

###################### AVOIDANCE ######################   
    av = avoidance()  
    kopterworx = CONTROLLER()
    # print(av.start_sim)
    # time.sleep(5)
    while not av.start_sim : 
        # print(av.start_sim)
        time.sleep(0.1)    
    while kopterworx.current_position.x <= av.goal_coordinate :
        x_desired = kopterworx.current_position.x + av.x_to_move
        y_desired = kopterworx.current_position.y + av.y_to_move
        kopterworx.waypoint(x_desired, y_desired, 0)

    # print('Avoidance Done')
############ SURVEYING AND MARKER DETECTION ############
    av.camera0.unregister()
    kopterworx.waypoint(2.5,-5.5,-1.57, 3)
    c = detector()
    ############### LOWER SCAN ###############
    while kopterworx.current_position.x < 10 :
        cv = c()
        if cv[0]:
            normal = np.array([0,-1,0])
            yaw = -np.pi/2
            break
        x_to_move = vel
        x_desired = kopterworx.current_position.x + x_to_move
        kopterworx.waypoint(x_desired, -5.5, -1.57, 3)
        #print('going')
   
    if not cv[0]:
        while kopterworx.yaw < 0 :
            yaw_desired = kopterworx.yaw  + angular_vel
            kopterworx.waypoint(10, -5.5, yaw_desired, 3)
            cv = c()
            if cv[0]:
                break
            
    if not cv[0]:
        while kopterworx.current_position.y < 5.5 :
            cv = c()
            if cv[0]:
                normal = np.array([1,0,0])
                yaw = 0
                break
            y_to_move = vel
            y_desired = kopterworx.current_position.y + y_to_move
            kopterworx.waypoint(10, y_desired, 0, 3)
    

    if not cv[0]:
        while kopterworx.yaw < 1.57 :
            yaw_desired = kopterworx.yaw  + angular_vel
            kopterworx.waypoint(10, 5.5, yaw_desired, 3)
            cv = c()
            if cv[0]:
                break

    if not cv[0]:
        while ((kopterworx.current_position.x > 2)and (kopterworx.current_position.y>0)):
            cv = c()
            if cv[0]:
                normal = np.array([0,1,0])
                yaw = np.pi/2
                break
            x_to_move = -vel
            x_desired = kopterworx.current_position.x + x_to_move
            kopterworx.waypoint(x_desired, 5.5, 1.57, 3)
    

    ########### UPPER SCAN #############  
    kopterworx.waypoint(2, 5.5, 1.57, 1.5)  
    while ((kopterworx.current_position.x < 10) and (kopterworx.current_position.y>0)) :
        cv = c()
        if cv[0]:
            normal = np.array([0,1,0])
            yaw = np.pi/2
            break
        x_to_move = vel
        x_desired = kopterworx.current_position.x + x_to_move
        kopterworx.waypoint(x_desired, 5.5, 1.57, 1.5)
  
    
    if not cv[0]:
        while kopterworx.yaw > 0 :
            yaw_desired = kopterworx.yaw  - angular_vel
            kopterworx.waypoint(10, 5.5, yaw_desired, 1.5)
            cv = c()
            if cv[0]:
                break
            
    if not cv[0]:
        while kopterworx.current_position.y > -5.5 :
            cv = c()
            if cv[0]:
                normal = np.array([1,0,0])
                yaw = 0
                break
            y_to_move = vel
            y_desired = kopterworx.current_position.y - y_to_move
            kopterworx.waypoint(10, y_desired, 0, 1.5)
    

    if not cv[0]:
        while kopterworx.yaw > -1.57 :
            yaw_desired = kopterworx.yaw  - angular_vel
            kopterworx.waypoint(10, -5.5, yaw_desired, 1.5)
            cv = c()
            if cv[0]:
                break

    if not cv[0]:
        while ((kopterworx.current_position.x > 2) and (kopterworx.current_position.y<0)) :
            cv = c()
            if cv[0]:
                normal = np.array([0,-1,0])
                yaw = -np.pi/2
                break
            x_to_move = -vel
            x_desired = kopterworx.current_position.x + x_to_move
            kopterworx.waypoint(x_desired, -5.5, -1.57, 1.5)
   

###################### BALL LAUNCH ####################

    icuas=FLIGHT_ICUAS()
    locmarker = np.array([cv[1].x, cv[1].y, cv[1].z]) # world
    # print("Coordinates:", locmarker)
    # print("Normal", normal)
    # print("Yaw", yaw)
    # print(cv)
    icuas.ball_drop(locmarker,normal,yaw)

################### THE END ##################
    

