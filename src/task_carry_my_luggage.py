'''
STAGE 1 TASK: CARRY MY LUGGAGE


1. start in living room
2. move to pointed luggage
3. pick up luggage (+100 if got correct one)
4. follow the operator
5. try to avoid obstacles
6. if cannot avoid, look for waving hands or smth idk bro
7. place the luggage in the car


8. go back into the arena??
9. queue????
'''




# run with conda env: nlp
import os
import signal
import roslib
import rospy
import smach
import smach_ros
import nlp_client
import threading
from ratfin import *
from core_smach.grasp_object import GraspObject
from core_smach.place_object import PlaceObject
from core_smach.move_to import Move_To

class MoveToLuggage(smach.State):
    def __init__(self, outcomes='out1', input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)
    
    def execute(self, ud):
        return 'out1'



def main():
    speak_debug = False
    response_debug = False
    NODE_NAME = "smach_task_receptionist"
    rospy.init_node(NODE_NAME)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out0','out1'])

    with sm:

        #TODO move to pointed luggage??? could try pose estimation to see which arm is lifted
        smach.StateMachine.add('MOVE_TO_LUGGAGE', 
                                MoveToLuggage(),
                                transitions={'out1':'GRASP_LUGGAGE'})

        #TODO if we can pick it up then fine, if not just wait til user place luggage on Walkie
        smach.StateMachine.add('GRASP_LUGGAGE',
                                GraspObject(),
                                transitions={'out1':'FOLLOW_USER'})
        

        #TODO FOLLOW ME FUNCTION I HERE AT THIS POINT WE SHOULD HAVE IT READY
        smach.StateMachine.add('FOLLOW_USER',
                               FOLLOW_ME_I_KUAY_I_NA_HEE(),
                               transitions={'out1':'HAND_LUGGAGE_OVER',
                                            'out2':'FIND_USER'})
        
        #TODO IF LOST MAKE A STATE TO FIND THE USER AGAIN THEN FOLLOW THEM AGAIN
        # DONT ASK ME HOW I STILL DONT KNOW FUCK ME FUCK ME FUCK ME
        smach.StateMachine.add('FIND_USER',
                               FIND_USER(),
                               transitions={'out1':'FOLLOW_USER'})

        #TODO HANDS LUGGAGE BACK, HOW TO IDK FUCK
        smach.StateMachine.add('HAND_LUGGAGE_OVER',
                               PlaceObject(),
                               transitions={'out1':'MOVE_BACK'})


        # TODO MOVE BACK INTO THE ARENA,QUEING UP BEHIND PEASANTS IS OPTIONAL,
        # BUT HOW TO IDK AGAIN BRO THIS SHIT HARD
        smach.StateMachine.add('MOVE_BACK',
                               Move_To(),
                               transitions={'out1':'out0'})




if __name__ == '__main__':
    main()