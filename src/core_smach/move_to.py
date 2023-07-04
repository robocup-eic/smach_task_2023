# State machine for move_to

import os
import signal
import roslib
import rospy
import smach
import smach_ros
import nlp_client
import threading
from ratfin import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion,Twist
from navigate_SEX import Navi_SEX

def print_available_userdata(userdata):
    print(userdata)

class MoveTo(smach.State):
    """
    TemplateVersion 1.1.0
    MoveTo state

    BASIC FUNCTION:
    Move from A to B with CV to find object/person
    1. [NLP] room, object, furniture, person, furniture_adjective,
        - [room, object, furniture, person] one must not be null
    2. [NAVIGATION] Move to the specified location
    3. [COMPUTER VISION] Detect the object/person. If not detected, repeat step 2
    3.1 [NLP] Announce "Unable to detect the object/person. Please try again" --> Loop
    4. [NLP] Announce "I have reached the location" --> out1

    """
    def __init__(self,
                 log : bool = False,
                 timeout_tries: int = 0, # 0 means infinite tries
                 target: str = None
                 ):
        """ 
        Assumption: only one object in the arena every object is unique
        MoveTo: fridge 
          """
        
        # Init smach
        smach.State.__init__(self,
                                outcomes=['out1','out0'],
                            #  outcomes=['out1','out2','loop','undo','timeout'],
                             input_keys=['room','furniture','data3'],
                             output_keys=['data1','data3'])
        rospy.init_node('nav_test', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        # Variable and Constants
        FILE_PATH =  ".yaml"


        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        if target is None:
            raise ValueError("Argument 'target_pose' must not be None")


        # action client for move_base
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        #allow up to 5 seconds for the action server to come up
        rospy.loginfo("wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries
        
        # Create instance of SEX to read the yaml for pose extraction
        sex = Navi_SEX(FILE_PATH)

        # get the Pose of the target location from SEX
        self.target_pose : Pose = sex.read_yaml(target)

    def execute(self, userdata):
        if self.target is None:
            rospy.logerr("Target not found")
            return 'out0'
        
        # A goal to be sent to the MoveBase action server.
        goal = MoveBaseGoal()
        # Reference frame
        goal.target_pose.header.frame_id = 'map'
        # header file 
        goal.target_pose.header.stamp = rospy.get_rostime()
        # take the extracted pose into 
        goal.target_pose.pose = self.target_pose

        # send goal to to movebase similar to post 
        self.move_base.send_goal(goal)
        rospy.loginfo("(MoveTo): Waiting for response from MoveBase")

        # maximum time to wait for the robot to reach the goal, returns true if it reaches the goal
        # returns false if it doesn't reach the goal
        success = self.move_base.wait_for_result(rospy.Duration(60))

        rospy.loginfo(f"(MoveTo): Result from MoveBase | {success}")

        # returns GoalStatus type of PENDING, ACTIVE, PREEMPTED, SUCCEEDED, ABORTED, REJECTED, PREEMPTING, and RECALLING.
        state = self.move_base.get_state()

        # safety procedure to stop the robot if it doesn't reach the goal
        self.shutdown()
        if success and state == actionlib.GoalStatus.SUCCEEDED:
            # We made it!
            return 'out1'
        else:
            # Cancel the goal if takes too long
            self.move_base.cancel_goal()
            return 'out0'
        


    def shutdown(self):
        stop_goal = MoveBaseGoal()
        self.move_base.send_goal(stop_goal)
        rospy.loginfo("(MoveTo) Stopping...")