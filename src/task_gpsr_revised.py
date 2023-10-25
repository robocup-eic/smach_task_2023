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
openai.api_key = "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"

import nlp_client as nlp
from core_nlp.emerStop import EmergencyStop, KillProcessState
from core_smach.gripper_control import GripperControl
from ssc_generator_state_final import SmachGeneratorState
from task_gpsr_states_dependancies import GTFO, SimGraspObjectGPSR, SimPlaceObjectGPSR, GoGo, GoToInstruction



ALL_OFFLINE = True
# Planning

# Idle General State Machine
""" 
Then respond by ChatGPT then pass the text to the NLU to get the intent and entities"""



# # print(nlp.get_intent("I want to go to the kitchen"))
# print(nlp.get_intent("Please go to to the kitchen and bring me a coke"))
# nlp.ww_listen()
# nlp_response = nlp.listen()




""" 
EXAMPLES ADDED TO RASA
EXAMPLES:



Meet William at the sink and follow him
Meet Charlie at the bed and follow him to the bedroom
Please give me the left most object from the cupboard
Tell me how many people in the dining room are boys
Robot please meet Charlie at the desk and follow her to the corridor
Please escort Patricia from the bed to the bookcase
Give the spoon to me
Follow Robin
Navigate to the couch, meet Alex, and take him to the entrance
Please meet Robert at the desk and follow him
Meet Jennifer at the end table, follow her, and escort her back
Please face Francis at the sink and take her to her taxi
Follow Patricia from the couch to the bedroom
Please tell the day of the week to the person pointing to the right in the living room
Meet Robin at the sink, follow him, and go to the kitchen
Could you place the bowl on the end table
Could you go to the bed, meet Robin, and accompany her to the end table
Take out the debris
Take the scrubby to the cupboard
Follow Linda from the sink to the living room

Could you Tell me the name of the person in the bedroom
Could you meet Elizabeth at the bookcase, follow her, and accompany her back
Robot please take out the junk
Meet Alex at the sink, follow him, and lead him back
Find Skyler at the sink and ask him to leave
Greet Robin at the bookcase and ask her to leave
Could you face Alex at the main door and introduce him to everyone in the dining room.
Please give me the right most object from the end table
Bring me the object above the knife from the dining table
Could you greet Skyler at the sink and ask her to leave
Go to the bookcase, meet Mary, and take her to the exit
Get the tray and place it on the desk
Tell me how many people in the living room are boys
Say your team's country to the person raising their right arm in the living room
Could you please give me the paprika from the end table
Tell me the gender of the person in the kitchen
Robot please tell me which are the three biggest snacks on the counter
Meet John at the couch, follow him, and navigate to the bedroom
Go to the bookcase, meet Patricia, and follow her
Robot please go to the dining table, meet Michael, and take him to the entrance

Go to the bedroom and grab a coke

! Intent that are still  is still sketch 
[0.082s] [0.0]{"intent": "what_is_your_name", "confidence": 0.21833959221839905, "text": "Say your team's country to the person raising their right arm in the living room", "rpos": "right", "room": "living room"}
[0.077s] [0.0]{"intent": "grasp_object", "confidence": 0.2994123101234436, "text": "Robot please take out the junk", "furniture": "junk"}
[0.071s] [0.0]{"intent": "place_object", "confidence": 0.8279561400413513, "text": "Please give me the left most object from the cupboard", "rpos": "left", "people": "me", "furniture": "cupboard"}
[0.072s] [0.0]{"intent": "place_object", "confidence": 0.99757319688797, "text": "Give the spoon to me", "object": "spoon", "people": "me"}
[0.077s] [0.0]{"intent": "place_object", "confidence": 0.9985761642456055, "text": "Could you place the bowl on the end table", "object": "bowl", "furniture": "end table"}
[0.093s] [0.0]{"intent": "place_object", "confidence": 0.34773197770118713, "text": "Take out the debris"}
[0.077s] [0.0]{"intent": "place_object", "confidence": 0.9990658164024353, "text": "Take the scrubby to the cupboard", "object": "scrubby", "furniture": "cupboard"}



ADDITIONAL STATE FOR GPSR
--------------------------
1. follow person
2. Meet person via CV
3. Escort person NO JUST NO
4. Find person PENDING with ICE

5. tell day of the week DONE
6. Ask for name DONE
7. ask him to leave (please follow me to the exit) DONE
8. tell the time DONE

list of possible intents
-----------------------
place_object
move_to+grasp_object+place_object
move_to+escort_people
what_is_their_name
move_to+ask_to_leave
object_counting
follow_people
what_is_your_name
move_to+follow_people
grasp_object


"""

""" 
Fundamental States
------------------------
GripperControl


List of State availiable for SSC later
------------------------
TellTime
TellDay
GTFO (ask him to leave)
GetName
SimGraspObjectGPSR




for intent rasa
move_to+
Could you Tell me the name of the person in the bedroom
 """

# Model Smach States


class IdleGeneral(smach.State):
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
        self.message = [{
            "role": "system", 
            "content" : """You’re a kind helpful assistant robot, This is a role-play
                            respond back to me what my commands were but rephrase it like a assistant would 
                            by accepting my request. don't ask a question back just do as I says. For example, 
                            if I ask you to retrieve a coke. You should respond with something like "Certainly, grabing you a coke now" but make the sentence dynamic dont actually use the word certainly its too formal. 
                            However if I ask you a factual question like "what is the weather today" you should respond with something like "The weather today is sunny"""}]
        
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','out0'],
                             input_keys=['intent','state_sequence'],
                             output_keys=['intent','state_sequence'])

        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries
        

        # Set self.variables

    def sendGPT(self, text):
        
        self.message.append({"role": "user", "content": text})
        
        completion = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=self.message
        )

        chat_response = completion.choices[0].message.content
        nlp_client.speak(chat_response)
        self.message.append({"role": "assistant", "content": chat_response})

    def execute(self, userdata):
        try:
            userdata.state_sequence = []
            # Log the execution stage
            rospy.loginfo(f'(IdleGeneral): Executing..')

            nlp.speak("I am entering idle, you can command me by saying 'Hey walkie' follow by your command.")
            rospy.loginfo(f'(WakeWord): Waiting for Hey Walkie')


            # wakeword blocker
            nlp.ww_listen()
            rospy.loginfo(f'(WakeWord): Triggered')

            while True:
                rospy.loginfo(f'(ASR): Listening')
                nlp_response = nlp.listen()
                rospy.loginfo(f'(ASR): {nlp_response.text}')
                if nlp_response.text != "":
                    break
            
            if ALL_OFFLINE:
                nlp_client.speak(nlp_response.text)
            else:
                # Text to be feed into ChatGPT and interface
                self.sendGPT(nlp_response.text)
            
            nlp_response # to extract intents

            userdata.intent = nlp_response.intent

            list_of_intents = userdata.intent.split("+")
            print(list_of_intents)
            for intent in list_of_intents:
                userdata.state_sequence.append(intent) 

            
            # take the intent and 

            
            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"   

def main():
    # Initialize the node
    NODE_NAME = "smach_task_gpsr_revised"
    rospy.init_node(NODE_NAME)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out0'])
    

    
    
    # Declear Variables for top-state
    sm.userdata.name = ""
    sm.userdata.intent = ""
    # sm.userdata.state_sequence = ['State1', 'State2','State1']
    sm.userdata.state_sequence = []
    """ 
    [0.081s] [0.0]{
        "intent": "move_to+grasp_object+place_object", "confidence": 0.9574416875839233, 
        "text": "Please give me the right most object from the end table", 
        "rpos": "right", 
        "people": "me", 
        "furniture_from": "end table"}
      """
    with sm:
        # smach.StateMachine.add('GO_MIDDLE',
        #                     GoToInstruction(),
        #                     transitions={'out1': 'IDLE_GENERAL',}
        #                                 )

        smach.StateMachine.add('IDLE_GENERAL',
                            # Speak(text="Please stand still for a moment while I scan you."),
                            IdleGeneral(),
                            remapping={'people_index': 'people_index'},
                            transitions={'out1': 'SMACH_GENERATOR',
                                        'out0': 'SMACH_GENERATOR'}
                                        )
        smach.StateMachine.add('SMACH_GENERATOR',
                            SmachGeneratorState(),
                            remapping={"state_sequence": "state_sequence"},
                            transitions={'out1': 'IDLE_GENERAL',
                                        'out0': 'IDLE_GENERAL'}
                                        )
        smach.StateMachine.add('KILLPROCESS',
                            KillProcessState(),
                            transitions={'out1': 'out0'}
                                        )
    
# Navigate to the coat rack, meet Angel, and follow her.


    # Create a thread to execute the smach container
    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    es = EmergencyStop()
    es_thread = threading.Thread(target=es.execute)
    es_thread.start()
    es.emer_stop_handler()

    
if __name__ == '__main__':
    main()

