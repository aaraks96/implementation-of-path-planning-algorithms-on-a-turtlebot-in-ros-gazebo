#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
import math
import ROS_test
import sys


class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        


    def move(self):
        
        start_x = int(input("Enter the start x position: "))*10
        start_y = int(input("Enter the start y position: "))*10
        goal_x = int(input("Enter the goal x position: "))*10
        goal_y = int(input("Enter the goal y position: "))*10
        velocity = [10,5]
        v = velocity
        r = 0.038
        l = 0.23
        vel_msg = Twist()
        count = 0
        while count<1:
            count+=1
            resolution = 1
            print("calling Astar")
            path,actions, angles= ROS_test.wrapper((start_x, start_y), [goal_x,goal_y], velocity, resolution)
            if path is None and actions is None and angles is None:
                print("Start or Goal point lies in obstacle space")
                print("Exiting program and shutting down Node")
                exit(0)
                
            if path ==0 and actions== 0 and angles==0:
                print("The goal node lies within the threshold")
                print("Exiting program")
                exit(0)
                
            actions = actions[::-1]
            print("PATH: ", path)
            print("ACTIONS: ", actions)
            for i in range(len(actions)):
                
                if actions[i] == "left_1":			
                    
                    vel_1 = 0 
                    vel_2 = v[0]
                if actions[i] == "right_1":			
                    
                    vel_1 = v[0]
                    vel_2 =  0
                if actions[i] == "left_2":
                    			
                    vel_1 = 0 
                    vel_2 = v[1]
                if actions[i] == "right_2":			
                    
                    vel_1 = v[1]
                    vel_2 =  0
                if actions[i] == "straight_1":			
                    
                    vel_1 = v[0]
                    vel_2 = v[0]
                if actions[i] == "straight_2":			
                    
                    vel_1 = v[1]
                    vel_2 = v[1]
                if actions[i] == "left_1_2":			
                    
                    vel_1 = v[0]
                    vel_2 = v[1]
                if actions[i] == "right_1_2":			
                    
                    vel_1 = v[1] 
                    vel_2 = v[0]
                
                vel_msg.linear.x = (r/2)*(vel_1 + vel_2) 
                vel_msg.linear.y = 0 
                vel_msg.linear.z = 0
                
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                
                vel_msg.angular.z = (r/l)*(vel_1 - vel_2)
                
                start_time = rospy.Time.now()
                duration = rospy.Duration(1)
                end_time = start_time + duration
                count = 0
                while rospy.Time.now() < end_time:
                    count +=1
                    self.velocity_publisher.publish(vel_msg)
                    self.rate.sleep()
            
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        x.move()

    except rospy.ROSInterruptException: pass
