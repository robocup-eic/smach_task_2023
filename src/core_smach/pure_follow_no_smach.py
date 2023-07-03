import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import *
import actionlib
from cv_connector.srv import CV_srv, CV_srvRequest
from cv_connector.msg import CV_type
import tf2_ros
from tf2_geometry_msgs import PoseStamped

def follow_person():
    # rospy.init_node("wahreever")
    cv_client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rate = rospy.Rate(5)
    
    rospy.wait_for_service('/CV_connect/req_cv')

    while not rospy.is_shutdown():
        print("request")
        req = CV_srvRequest()
        req.cv_type.type = CV_type.TargetTracker_Detect
        response = cv_client(req)
        print(response.result)
        rate.sleep()

    # move_base_client.wait_for_server()

    # goal= MoveBaseGoal()
    # goal.target_pose.header.frame_id = "base_link"
    # goal.target_pose.header.stamp = rospy.Time.now()
    # goal.target_pose.pose.position.x = 0.5
    # goal.target_pose.pose.position.y = 0.5
    # goal.target_pose.pose.orientation.w = 1.0

    # move_base_client.send_goal(goal)

    
    # print("Goal sent")

    # return client.get_state()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        follow_person()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
