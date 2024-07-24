#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion

class DockingNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('docking_node')
        rospy.sleep(5)
        self.t0 = rospy.get_time()

        # Subscriber to the tag pose
        self.pose_sub = rospy.Subscriber('/tag_pose', PoseStamped, self.pose_callback)

        # Publisher for velocity commands
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.twist=Twist()

         # Desired docking distance from the tag (in meters)
        self.docking_distance = 1

        # Tolerance for initial pose (in meters)
        self.start_tolerance = 0.5
        # Tolerance for docking (in meters)
        self.docking_tolerance = 0.05
        self.driving_speed = 0.5
        self.driving_speed_slow = 0.2
        self.driving_speed_approach = 0.025

        rospy.loginfo("Docking node initialized")

        rospy.sleep(0.5)
    
        
        

    def pose_callback(self, msg):
        # Extract the position of the tag
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        delta_x = msg.pose.position.x #positive is backward from dock
        delta_y = msg.pose.position.y #positive is left of dock
        delta_angle = yaw #negative pi is slightly left, positive pi is slightly right
        
        t1 = rospy.get_time() - self.t0
        
        # if t1>100:
        #     rospy.loginfo("Docking allowed time exceeded: " + str(t1))
        #     rospy.signal_shutdown("Docking allowed time exceeded")

        if self.docking_distance-self.start_tolerance<=delta_x<=self.docking_distance+self.start_tolerance:
            if delta_x>0.3:
                    self.twist.linear.x = self.driving_speed
            elif 0.3>=delta_x>0.15:
                self.twist.linear.x = self.driving_speed_slow
            elif 0.15>=delta_x<=self.docking_tolerance:
                self.twist.linear.x = self.driving_speed_approach
            else:
                self.twist.linear.x = 0
        
            
            self.cmd_pub.publish(self.twist)
            
             
           
            

      

if __name__ == '__main__':
    try:
        DockingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass