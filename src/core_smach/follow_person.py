
# ros libraries
import roslib
import rospy
import smach
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped, Point
from std_msgs.msg import String

# import computer vision and realsense
import numpy as np

#Bring in the simple action client
import actionlib

#Bring in the .action file and messages used by the move based action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from util.custom_socket import CustomSocket
from cv_bridge import CvBridge, CvBridgeError #
from sensor_msgs.msg import Image, CameraInfo #
from actionlib_msgs.msg import GoalStatus

# Utils function
from math import atan, pi

class FollowPerson(smach.State):
    def __init__(self):
        # init outomes
        smach.State.__init__(self, outcomes=['out1'])

        #init node
        rospy.loginfo('Initiating state Follow_person')

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # transform
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # stop robot publisher 
        self.cancel = Twist()
        self.stop_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=1)
        self.cancel.linear.x = 0
        self.cancel.linear.y = 0
        self.cancel.angular.z = 0

        # follow with whole body
        self.follow_cmd = String()
        self.follow_cmd_pub = rospy.Publisher("/human/follow_cmd", String, queue_size=1)
        self.follow_cmd.data = "follow"
        self.is_cancelled = False

        # init variables

        self.target_lost = False
        self.is_stop = False
        self.last_pose = None
        self.robot_inside = False

    def execute(self, userdata):
        rospy.loginfo('Executing state Follow_person')
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown(): # safer to use rospy.is_shutdown() instead of True | thanks copilot
            if self.robot_inside: # if robot is inside the map/ near door
                self.stop_following()
                self.follow_person()
            else:
                self.stop_following()
                self.handle_outside_map()

            if self.is_stop:
                return "continue_stop"

            rate.sleep()

    def follow_person(self):
        try:
            pose = self.get_person_pose()
            if pose is None:
                self.target_lost = True
                return

            if self.should_send_goal():
                self.send_goal(pose)

        except Exception as e:
            rospy.loginfo(e)
            self.target_lost = True

    def get_person_pose(self):
        pose = None
        try:
            pose = self.tf_buffer.lookup_transform('base_footprint', 'human_frame', rospy.Time.now() - rospy.Duration.from_sec(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        return pose

    def should_send_goal(self):
        if self.target_lost:
            self.client.cancel_goal()
            self.stop_pub.publish(self.cancel)
            speak("I have lost you, where are you my friend.")
            return False

        if self.last_pose is None:
            return True

        delta_x = pose.transform.translation.x - self.last_pose[0]
        delta_y = pose.transform.translation.y - self.last_pose[1]
        distance = math.sqrt(delta_x ** 2 + delta_y ** 2)

        if distance > 0.5:
            return True

        return False

    def send_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_footprint"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1.0)
        goal.target_pose.pose.position.x = pose.transform.translation.x
        goal.target_pose.pose.position.y = pose.transform.translation.y

        delta_x = pose.transform.translation.x - self.last_pose[0]
        delta_y = pose.transform.translation.y - self.last_pose[1]
        yaw = math.atan2(delta_y, delta_x)

        quarternion_orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = quarternion_orientation[0]
        goal.target_pose.pose.orientation.y = quarternion_orientation[1]
        goal.target_pose.pose.orientation.z = quarternion_orientation[2]
        goal.target_pose.pose.orientation.w = quarternion_orientation[3]

        self.client.send_goal(goal)
        self.last_pose = (pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z)

    def stop_following(self):
        self.follow_cmd.data = "stop" 
        self.follow_cmd_pub.publish(self.follow_cmd) # tell node to stop following

        if self.client.get_state() == GoalStatus.ACTIVE: #! WE DONT USE THIS SHIT NO MORE...
            self.client.cancel_goal()

        self.stop_pub.publish(self.cancel) # send twist 000

    def handle_outside_map(self):
        speak("Out of map")

        if not self.is_cancelled:
            self.client.cancel_goal()
            self.follow_cmd_pub.publish("follow")
            self.is_cancelled = True

        if self.target_lost:
            self.follow_cmd.data = "stop"
            self.follow_cmd_pub.publish(self.follow_cmd)
            self.stop_pub.publish(self.cancel)
            speak("I lost you, where are you my friend.")