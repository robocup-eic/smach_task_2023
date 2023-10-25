
import rospy
import smach
import threading
from core_nlp.emerStop import EmergencyStop
import subprocess
import smach
import nlp_client
from ratfin import *
import sys

from cv_connector.msg import CV_type

from cv_connector.srv import CV_srv, CV_srvRequest, CV_srvResponse
from dy_custom.srv import SetDigitalGripper, SetDigitalGripperRequest, SetDigitalGripperResponse, SetDegreeRequest,SetDegree,SetDegreeResponse
from core_nlp.utils import WakeWord , GetIntent
from core_smach.follow_person import Follow_Person
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

# Safety
from core_nlp.emerStop import EmergencyStop, KillProcessState
# smach state locations
from core_nlp.general import TellTime, TellDay, Pariotism
from core_nlp.utils import WakeWord, Speak, GetEntities, GetIntent, GetName, GetObject, GetLocation
from core_smach.follow_person import Follow_Person
from task_gpsr_states_dependancies import GTFO, SimGraspObjectGPSR, SimPlaceObjectGPSR, GoGo, GoToInstruction
from ssc_og import State1, State2

NODE_NAME = "auto_generated_smach"
rospy.init_node(NODE_NAME)

# Create a SMACH state machine
sm = smach.StateMachine(outcomes=['out0'])

# Declare Variables for top-state
sm.userdata.name = ""
sm.userdata.furniture = ""
sm.userdata.people = ""
sm.userdata.rpos = ""
sm.userdata.object = ""
sm.userdata.room = ""

with sm:

            
    smach.StateMachine.add('move_to_1',
                           GoGo(),
                           remapping={'data1': 'data1', 'data2': 'data2'},
                           transitions={'out1': 'follow_people_2', 'out0': 'follow_people_2'})
                
    smach.StateMachine.add('follow_people_2',
                           Follow_Person(),
                           remapping={'data1': 'data1', 'data2': 'data2'},
                           transitions={'out1': 'KILLPROCESS', 'out0': 'move_to_1'})
                
                    
    smach.StateMachine.add('BACK_START',
                            GoToInstruction(),
                            transitions={'out1': 'out0'}
                                        )
    smach.StateMachine.add('KILLPROCESS',
                            KillProcessState(),
                            transitions={'out1': 'out0'}
                                        )
    
# Create a thread to execute the smach container
smach_thread = threading.Thread(target=sm.execute)
smach_thread.start()

es = EmergencyStop()
es_thread = threading.Thread(target=es.execute)
es_thread.start()
es.emer_stop_handler()
            