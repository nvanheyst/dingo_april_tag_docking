#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion

class AprilTagPoseEstimator:
    def __init__(self):
        # Initialize the node
        rospy.init_node('april_tag_pose_estimator')

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        # Subscribe to the AprilTag detection topic
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        # Publisher for the tag pose
        self.pose_pub = rospy.Publisher('/tag_pose', PoseStamped, queue_size=10)

        # Variable to store the tag pose
        self.tag_pose = None

        rospy.loginfo("AprilTag Pose Estimator node initialized")

    def image_callback(self, data):
        # Convert the ROS image message to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
             rospy.logerr(e)

        # Process the image if tag_pose is available
        if self.tag_pose:
            # Draw the pose on the image (for visualization purposes)
            cv2.putText(cv_image, f"Tag Pose: x={round(self.tag_pose.position.x,2)}, y={round(self.tag_pose.position.y,2)}, z={round(self.current_yaw, 2)}", 
                        (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Image with AprilTag Pose", cv_image)
            cv2.waitKey(3)

    def tag_callback(self, data):
        # Check if any tags are detected
        if len(data.detections) > 0:
            for detection in data.detections:
                if detection.id[0] == 0:  # Check if the tag ID is 36_11_00000
                    # Extract the pose
                    pose = detection.pose.pose.pose

                    # Correct the coordinate frame
                    corrected_pose = PoseStamped()
                    corrected_pose.header.stamp = rospy.Time.now()
                    corrected_pose.header.frame_id = "camera_link"  # or the appropriate frame
                    corrected_pose.pose.position.x = pose.position.z
                    corrected_pose.pose.position.y = pose.position.x
                    corrected_pose.pose.position.z = pose.position.y
                    corrected_pose.pose.orientation.x = pose.orientation.z
                    corrected_pose.pose.orientation.y = pose.orientation.x
                    corrected_pose.pose.orientation.z = pose.orientation.y

                    orientation_q = pose.orientation
                    orientation_list = [orientation_q.z, orientation_q.x, orientation_q.y, orientation_q.w]
                    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                    self.current_yaw = yaw

                    # Store the corrected pose
                    self.tag_pose = corrected_pose.pose

                    # Publish the corrected pose
                    self.publish_pose(corrected_pose.pose)
                    rospy.loginfo(f"Tag ID 36_11_00000 detected. Corrected Pose: {self.tag_pose}")



    def publish_pose(self, pose):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "camera_link"  # or the appropriate frame
        pose_msg.pose = pose
        self.pose_pub.publish(pose_msg)

if __name__ == '__main__':
    try:
        AprilTagPoseEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
