import rospy
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import *
import actionlib
from cv_connector.srv import CV_srv, CV_srvRequest
from cv_connector.msg import CV_type
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from tf_manager.tf_main import transform_pose
from cv_computer.cv_computer import cv_computer
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def follow_person():
    cv_client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)
    rospy.wait_for_service('/CV_connect/req_cv')
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rate = rospy.Rate(5)
    

    req = CV_srvRequest()
    req.cv_type.type = CV_type.TargetTracker_Register
    response = cv_client(req)
    while response.result == "No person detect":
        print("No person register, try again")
        req = CV_srvRequest()
        req.cv_type.type = CV_type.TargetTracker_Register
        # req.req_info = "name"
        response = cv_client(req)
    
    s = cv_computer()
    rospy.wait_for_service('/CV_connect/req_cv')
    while not rospy.is_shutdown():
        human_pos = s.xyz_target()
        if human_pos == None:
            print("No person detect, try again")
            continue
        elif math.isnan(human_pos[0]) or math.isnan(human_pos[1]) or math.isnan(human_pos[2]):
            continue
        else:
            print(type(human_pos[0]),human_pos[0])
            pose_stamped = Pose()
            pose_stamped.position.x = human_pos[0]
            pose_stamped.position.y = human_pos[1]

            yaw = math.atan2(human_pos[1], human_pos[0])
            quarternion_orientation = quaternion_from_euler(0, 0, yaw)
            pose_stamped.orientation.x = quarternion_orientation[0]
            pose_stamped.orientation.y = quarternion_orientation[1]
            pose_stamped.orientation.z = quarternion_orientation[2]
            pose_stamped.orientation.w = quarternion_orientation[3]
            s.pub_marker(human_pos[0], human_pos[1], human_pos[2], "base_link")

            move_base_client.wait_for_server

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "base_link"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = pose_stamped

            move_base_client.send_goal(goal)

        rate.sleep()


        

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        follow_person()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
