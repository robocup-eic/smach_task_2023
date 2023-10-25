import subprocess
import smach
import nlp_client
from ratfin import *
# add path up one directory
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

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
from core_nlp.emerStop import EmergencyStop
# smach state locations
from core_nlp.general import TellTime, TellDay, Pariotism
from core_nlp.utils import WakeWord, Speak, GetEntities, GetIntent, GetName, GetObject, GetLocation
from core_smach.follow_person import Follow_Person
# from task_gpsr_revised import GTFO, IdleGeneral, SimGraspObjectGPSR, SimPlaceObjectGPSR, ExampleState, GoGo
from task_gpsr_states_dependancies import GTFO, SimGraspObjectGPSR, SimPlaceObjectGPSR, GoGo, GoToInstruction
from ssc_og import State1, State2
"""

List of State availiable for SSC later
------------------------
TellTime
TellDay
GTFO (ask him to leave)
GetName
SimGraspObjectGPSR
SimPlaceObjectGPSR
"""
class SmachGeneratorState(smach.State):
    def __init__(self):
        """ input_keys=['state_sequence']
         userdata.state_sequence = ['State1', 'State2','State1'] """
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['state_sequence'],
                             output_keys=['execution_result'])

        # Mapping of state names to classes (not instances of the classes)
        self.state_mapping = {
            'State1': State1,
            'State2': State2,
            # Add more states here...
        }
        self.state_mapping = {
            'patriotism': Pariotism,
            'tell_time': TellTime,
            'tell_day': TellDay,
            'ask_to_leave': GTFO,
            'grasp_object': SimGraspObjectGPSR,
            'what_is_your_name': GetName,
            'place_object': SimPlaceObjectGPSR,
            'move_to': GoGo, # FIX THIS LATER
            'object_counting': TellTime,
            'follow_people': Follow_Person,
            'escort_people': Follow_Person,
            'what_is_their_name': GetName,}

    def generate_state_dict_list(self, state_sequence):
        state_dict_list = []

        for i in range(len(state_sequence)):
            current_state = state_sequence[i]
            state_obj = self.state_mapping[current_state]()  # Instantiate the state class
            current_state_with_suffix = current_state + "_" + str(i + 1)  # appending the suffix

            # Define remappings here
            remappings = {'data1': 'data1', 'data2': 'data2'}  # Use actual remappings


            if i == len(state_sequence) - 1:  # Last state
                transitions = {'out1': 'KILLPROCESS',
                            'out0': state_sequence[i - 1] + "_" + str(i) if i > 0 else 'KILLPROCESS'}
            else:
                transitions = {'out1': state_sequence[i + 1] + "_" + str(i + 2),
                            'out0': state_sequence[i - 1] + "_" + str(i) if i > 0 else state_sequence[i + 1] + "_" + str(i + 2)}

            state_dict_list.append({
                'state_name': current_state_with_suffix,
                'state_obj': type(state_obj).__name__,
                'transitions': transitions,
                'remappings': remappings  # Added remappings here
            })

        return state_dict_list



    # def construct_smach_state_machine(self, states_dict_list):
    #     sm = smach.StateMachine(outcomes=['out0'])

    #     with sm:
    #         for state_dict in states_dict_list:
    #             smach.StateMachine.add(state_dict['state_name'],
    #                                 state_dict['state_obj'],
    #                                 transitions=state_dict['transitions'])

    #     return sm

    def execute(self, userdata):
        rospy.loginfo(f'(SmachStateGenerator): state_sequence = {userdata.state_sequence}')
        state_sequence = userdata.state_sequence
        rospy.loginfo(f'(SmachStateGenerator): sequencuing..')
        state_dict_list = self.generate_state_dict_list(state_sequence)
        # sm = self.construct_smach_state_machine(state_dict_list)

        # Write to a file
        rospy.loginfo(f'(SmachStateGenerator): tryingWriting to the file..')
        filename = "temp_generated_smach_state.py"
        rospy.loginfo(f'(SmachStateGenerator): Writing to the file..')
        with open(filename, 'w') as f:
            f.write("""
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

            """)

            # Dynamically add states to smach
            for state in state_dict_list:
                f.write("""
    smach.StateMachine.add('{0}',
                           {1}(),
                           remapping={2},
                           transitions={3})
                """.format(state['state_name'], state['state_obj'], state['remappings'], state['transitions']))

            # Add EmergencyStop code to the file
            f.write("""

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
            """)

        # Execute the generated script
        rospy.loginfo(f'(SmachStateGenerator): Wrote sucessfully. Executing the script..')
        try:
            subprocess.run(["python3", filename], check=True)
            userdata.execution_result = "out1"
            return 'out1'
        except subprocess.CalledProcessError:
            userdata.execution_result = "out0"
            return 'out0'


def main():
    # Initialize the node
    NODE_NAME = "TestSmachGenerator"
    rospy.init_node(NODE_NAME)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out0'])
    # Declear Variables for top-state
    # sm.userdata.name = ""
    # sm.userdata.intent = ""
    sm.userdata.state_sequence = ['State1', 'State2','State1']
    with sm:

        smach.StateMachine.add('TEST_GENERATOR',
                            SmachGeneratorState(),
                            remapping={"state_sequence": "state_sequence"},
                            transitions={'out1': 'out0',
                                        'out0': 'out0'}
                                        )



    es = EmergencyStop()


    # Create a thread to execute the smach container
    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    es_thread = threading.Thread(target=es.execute)
    es_thread.start()
    es.emer_stop_handler()


if __name__ == '__main__':
    main()


