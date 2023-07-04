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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import smach
from nlp_client import ww_listen
from threading import Thread

class Follow_Person(smach.State):
    def __init__(self,target_name:str = 'target', timeout_seconds : int = 10 ):
        self.target_name = target_name
        self.timeout_seconds = timeout_seconds
        self.reached_destination = False
        smach.State.__init__(self, outcomes=['out1', 'out2'])

    def execute(self, userdata):
        rospy.loginfo(('(FollowPerson) Executing following person'))
        return self.follow_person()

    def check_reach_destination(self):
        ww_listen()
        self.reached_destination = True

    def follow_person(self):
        # client for CV services
        rospy.loginfo('(FollowPerson) Waiting for CV service')
        cv_client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)
        rospy.wait_for_service('/CV_connect/req_cv')

        # client for sending goals to move_base
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rate = rospy.Rate(5)
        
        rospy.loginfo('(FollowPerson) Registering person')
        req = CV_srvRequest()
        req.cv_type.type = CV_type.TargetTracker_Register
        # req.req_info = self.target_name # name of person to follow
        response = cv_client(req)
        
        while response.result == "No person detect":
            rospy.loginfo("No person register, try again")
            req = CV_srvRequest()
            req.cv_type.type = CV_type.TargetTracker_Register
            # req.req_info = self.target_name # name of person to follow
            response = cv_client(req)
        
        # will be used to compute values from the camera
        s = cv_computer()
        rospy.wait_for_service('/CV_connect/req_cv')
        
        #create a thread to check if the robot has reached the destination
        are_we_there_yet_bitch = Thread(target=self.check_reach_destination)
        rospy.loginfo('(FollowPerson) Waiting for WakeWord to stop')
        are_we_there_yet_bitch.start()

        while not rospy.is_shutdown() and not self.reached_destination:
            # will return xyz coordinates relative to the robot for the target - default param is target = 'target'
            human_pos = s.xyz_target()
            # rospy.loginfo('(FollowPerson) Detected person')
            while human_pos == None: # human not detected
                rospy.loginfo('(FollowPerson) Cannot detect anyone')
                timeout_count = 0
                if timeout_count > self.timeout_seconds/(1/5): # means that person has not been detected for longer than wanted
                    return 'out2' # person missing, transition to find person
                human_pos = s.xyz_target()
                timeout_count +=1
                rate.sleep()    

            if math.isnan(human_pos[0]) or math.isnan(human_pos[1]) or math.isnan(human_pos[2]):
                continue
            else:
                print(type(human_pos[0]),human_pos[0])
                #create a Pose object relative to the base_link frame to be sent to move_base
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

                move_base_client.wait_for_server()

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "base_link"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = pose_stamped

                move_base_client.send_goal(goal)

            rate.sleep()
        are_we_there_yet_bitch.join()
        return 'out1'
    

def main():
    NODE_NAME = "smach_test_follow_person"
    rospy.init_node(NODE_NAME)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out0','out1'])

    with sm:
        smach.StateMachine.add('FOLLOW_ME', 
                                Follow_Person(),
                                transitions={'out1':'out1',
                                             'out2':'out0'})
    sm.execute()



if __name__ == '__main__':
   main()