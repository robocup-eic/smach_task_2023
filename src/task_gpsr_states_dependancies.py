import signal
import roslib
import rospy
import smach
import smach_ros
import nlp_client
import threading
from ratfin import *
import time
import os
import threading
from dy_custom.srv import SetDigitalGripper, SetDigitalGripperRequest, SetDigitalGripperResponse, SetDegreeRequest,SetDegree,SetDegreeResponse
import openai
import yaml

from geometry_msgs.msg import Pose
import nlp_client as nlp
from core_nlp.emerStop import EmergencyStop
from core_smach.gripper_control import GripperControl
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class SimGraspObjectGPSR(smach.State):
    
    """ 
     The robot is unable to grab the object, the robot will ask the ref to grab the object """
    def __init__(self, 
                 log : bool = False,
                 timeout_tries: int = 0 # 0 means infinite tries
                 ):
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','out0'],
                             input_keys=['data1','data2','data3'],
                             output_keys=['data1','data3'])

        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries

        # Gripper stuff
        self.called = False
        self.client = rospy.ServiceProxy("/dy_custom/gripper/set_digital",SetDigitalGripper)
        # wait for the service to be ready
        self.client.wait_for_service()
        rospy.loginfo("(GripperControl) Gripper service is ready")

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(SimGraspObjectGPSR): Executing..')

            # Userdata verification
            rospy.loginfo(f'(SimGraspObjectGPSR): Checking userdata..')

            # Do something
            print(userdata)

            nlp_client.speak("""I have arrived at the object to be grab, I am unable to grab the object.
                              Could the referee grab the object and place it on my arm and say Hey Walkie for me to grab the object. 
                             I am opening my hand now""")
            
            # Opening gripper
            rospy.loginfo("(GripperControl) Opening gripper")
            self.client.call(SetDigitalGripperRequest(-1))
            rospy.sleep(2)


            # add a while loop to wait for the ref to grab the object with a wakeword blocker with 10 seconds timeout
            # Create a Thread object
            while True:

                listen_thread = threading.Thread(target=nlp_client.ww_listen)

                # Start the thread, this will initiate ww_listen function in a separate thread
                listen_thread.start()

                # Wait for 7 seconds or until the thread finishes, whichever comes first
                listen_thread.join(timeout=4)

                if listen_thread.is_alive():
                    print('Wakeword not detected within 7 seconds.')
                    # If necessary, add code here to stop the ww_listen() function
                    nlp_client.speak("I repeat if when you are ready to place the object on my arm, please say Hey Walkie and I will grip it")
                else:
                    print('Wakeword detected!')
                    break
            
            # Closing gripper
            nlp_client.speak("I am closing my hand now")
            rospy.loginfo("(GripperControl) Closing gripper")
            self.client.call(SetDigitalGripperRequest(1))
            
            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"

class SimPlaceObjectGPSR(smach.State):
    """ 
     The robot is unable to grab the object, the robot will ask the ref to grab the object """
    def __init__(self, 
                 log : bool = False,
                 timeout_tries: int = 0 # 0 means infinite tries
                 ):
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','out0'],
                             input_keys=['data1','data2','data3'],
                             output_keys=['data1','data3'])

        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries

        # Gripper stuff
        self.called = False
        self.client = rospy.ServiceProxy("/dy_custom/gripper/set_digital",SetDigitalGripper)
        # wait for the service to be ready
        self.client.wait_for_service()
        rospy.loginfo("(GripperControl) Gripper service is ready")

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(SimPlaceObjectGPSR): Executing..')

            # Userdata verification
            rospy.loginfo(f'(SimPlaceObjectGPSR): Checking userdata..')

            # Do something
            print(userdata)

            nlp_client.speak("""I have retrived the object you have request. Letting go in 3. 2. 1.""")
            
            # Opening gripper
            rospy.sleep(1)
            rospy.loginfo("(GripperControl) Opening gripper")
            self.client.call(SetDigitalGripperRequest(-1))
            
            # Closing gripper
            nlp_client.speak("I am closing my hand now")
            rospy.loginfo("(GripperControl) Closing gripper")
            self.client.call(SetDigitalGripperRequest(1))
            
            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"

class ExampleState(smach.State):
    """ 
    TemplateVersion 1.1.0 
    """
    def __init__(self, 
                 log : bool = False,
                 timeout_tries: int = 0 # 0 means infinite tries
                 ):
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1', 'out0'],
                             input_keys=['data1'],
                             output_keys=['data1'])

        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False

    def execute(self, userdata):
        # Log the execution stage
        rospy.loginfo(f'(ExampleState): Executing..')
        return "out1"


class GTFO(smach.State):
    def __init__(self, 
                 log : bool = False,
                 timeout_tries: int = 0 # 0 means infinite tries
                 ):
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','out0'],
                             input_keys=['name'],)

        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries

        # Set self.variables

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(GetTheFuckOut): Executing..')

            # Userdata verification
            rospy.loginfo(f'(GetTheFuckOut): Checking userdata..')


            #TODO Find the person to track 
            if (userdata.name != None) and (userdata.name != ""):
                nlp_client.speak(f"""Hello I am assuming you are {userdata.name}. 
                                 Nice to meet you the host has ask me to ask you to get the hell out of their house right now.
                                 No offense but the owner just doesn't like you. I believe you know where the door is""")
            else:
                nlp_client.speak(f"""Hello the host has ask me to ask you to get the hell out of their house right now.
                                 No offense but the owner just doesn't like you. I believe you know where the door is""")

            # Do something
            print(userdata)

            while (self.tries_counter < self.timeout_tries) or not self.timeout_bool:
                # Increment the counter
                self.tries_counter += 1
            
            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"


class GoGo(smach.State):
    
    """ This is the move to function for the robot   """
    def __init__(self):
        smach.State.__init__(self,outcomes=['out1','out0'],input_keys=['room'])
        self.yaml_reader = read_yaml()

    
    def execute(self, ud):
        return self.go_to_person(ud.room)
    
    def go_to_person(self,room):
        rospy.loginfo("Going to places")
        pose_list = self.yaml_reader.find("find_person", room_name=room)
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        pose = pose_list[0]
        move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose.position.x
        goal.target_pose.pose.position.y = pose.position.y
        goal.target_pose.pose.position.z = pose.position.z

        goal.target_pose.pose.orientation.x = pose.orientation.x
        goal.target_pose.pose.orientation.y = pose.orientation.y
        goal.target_pose.pose.orientation.z = pose.orientation.z
        goal.target_pose.pose.orientation.w = pose.orientation.w

        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()
        return 'out1'

class read_yaml :

    def __init__(self) :
        self.f = open('/home/walkie/smach_task_2023/src/station_pos.yaml')
        self.docs = yaml.load_all(self.f, Loader=yaml.FullLoader)
        self.pose_list = []

    def find(self, find_type, room_name) :
        self.pose_list.clear()
        ret = []
        pos = Pose()
        for doc in self.docs :
            if(find_type in doc) :
                dict_room = doc[find_type]
                list_pos_key = dict_room[room_name].keys()
                for pose in list_pos_key :
                    self.pose_list.append(dict_room[room_name][pose])
        for pose in self.pose_list :
            pos.position.x = pose["position"]["x"]
            pos.position.y = pose["position"]["y"]
            pos.position.z = pose["position"]["z"]
            pos.orientation.x = pose["orientation"]["x"]
            pos.orientation.y = pose["orientation"]["y"]
            pos.orientation.z = pose["orientation"]["z"]
            pos.orientation.w = pose["orientation"]["w"]
            ret.append(pos)
        return ret
    
class GoToInstruction(smach.State):
    """ This is the move to function for the robot   """
    def __init__(self):
        smach.State.__init__(self,outcomes=['out1','out0'])
        self.yaml_reader = read_yaml()
    
    def execute(self, ud):
        return self.go_to_person()
    
    def go_to_person(self):
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 0.18838489055633545
        goal.target_pose.pose.position.y = 1.5585248470306396
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0.70411130653909
        goal.target_pose.pose.orientation.w = 0.7100896196986798

        print("GOOOOOOOO")
        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()
        return 'out1'

