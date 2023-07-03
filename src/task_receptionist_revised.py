# run with conda env: nlp
import os
import signal
import roslib
import rospy
import smach
import smach_ros
import threading
import sys
import nlp_client
import json
import ast
from ratfin import *
from geometry_msgs.msg import Twist, Vector3
from rospy import Publisher, Rate

from core_smach.person import Person
from core_nlp.utils import (WakeWord, Speak, GetIntent, GetName, GetObject, GetLocation,)
from core_smach.move_to import MoveTo
from core_cv.image_captioning import ImageCaption
from cv_connector.srv import CV_srv, CV_srvRequest
from cv_connector.msg import CV_type



""" Overall Flow 
1. Wait in the room
1.5 (Optional) Open the door the guest
2. When Person1 enters the room, greet them "Greetings, May I have your name please?" 
2.5 Turn towards Person1 (Remain eye contact)
3. Wait for the person to respond with their name
4. When the person responds with their name, greet them with their name and ask them "What is your favorite drink?"
5. Wait for the person to respond with their favorite drink
6. Then ask them to stand still for a moment while you scan them
7. Scan the person
8. When the scan is complete
9. Look towards the couch
10. Say "Please take a sit on the couch, This is our host "Game" his favorite drink is "Coke". Game this is Person1 etc....
11. Wait for the person to sit down
12. Back to the idle position
13 When Person2 enters the room, greet them "Greetings, May I have your name please?" 
14 Turn towards Person1 (Remain eye contact)
15 Wait for the person to respond with their name
16 When the person responds with their name, greet them with their name and ask them "What is your favorite drink?"
17 Wait for the person to respond with their favorite drink
18 Then ask them to stand still for a moment while you scan them
19 Scan the person
20 When the scan is complete
21 Look towards the couch
22 Say "Please take a sit on the couch, This is our host "Game" his the in the middle of the couch, his favorite drink is "Coke". Person1 name this is Person2 etc....

"""




# Task specific state
class AddPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['out1','out0'],
                            input_keys=['name','favorite_drink','age',
                                         'shirt_color','hair_color',
                                         'people_list', 'people_index'],
                            output_keys=['people_list','people_index'])

    def execute(self, userdata):

        # Log the execution stage
        rospy.loginfo(f'(AddPerson): Executing..')
        
        p = Person(name=userdata.name,
                   favorite_drink=userdata.favorite_drink,
                   age=userdata.age,
                   shirt_color=userdata.shirt_color,
                   hair_color=userdata.hair_color
                   )
        
        # Add person object to people_list
        userdata.people_list.append(p)

        # print all people attributes
        # for person in userdata.people_list:
        #     print(person.__dict__)

        userdata.people_index += 1
        rospy.loginfo(f'(AddPerson): {p.name} added to people_list. {p.__dict__}')
        
        return 'out1'

# Task specific state
class IntroducePeople(smach.State):
    def __init__(self,
                 people_index : int = 0, 
                 header_text : str = "", 
                 introduce_to : int = None,
                 person_name_to_track : str  = None, # track the listener while speaking
                 closet_person_to_track : bool = False, # track the closet person while speaking
                 log : bool = True,
                 ):
        smach.State.__init__(self,
                            outcomes=['out1'],
                            input_keys=['people_list', 'people_index'],
                            output_keys=['people_index'])
        self.index = people_index
        self.header_text = header_text
        self.introduce_to = introduce_to
        self.person_name_to_track = person_name_to_track
        self.closet_person_to_track = closet_person_to_track
        self.log = log


    def person_tracker(self, 
                    #    userdata, 
                       continous: bool = True ):
        """ 
        ## To Run 
            1. rosrun cv_connector CV_connector.py
            2. cv_main.py must be running 
            3. roslaunch zed_wrapper zed2i.launch
            4. rosservice call /CV_connect/req_cv "cv_type: type: 1 " """
        def find_by_name_return_center(data, name):
                    """ 
                    Return [X, Y]
                    >>> x = find_by_name_return_center(detection_list, "Game"):
                    x = [757,380]

                    """
                    for key, value in data.items():
                        if value.get('name') == name:
                            
                            return value["center"]
                    return None
        def find_by_closest_distance_center(data):
                    """ 
                    Return [X, Y]
                    >>> x = find_by_closest_distance_center(detection_list):
                    x = [757,380]

                    """
                    sorted_data = dict(sorted(detection_list.items(), key=lambda item: item[1]['area'], reverse=True))
                    if self.response_debug:
                        rospy.loginfo("sorted_data: ")
                        printclr(json.dumps(sorted_data, indent=4),"blue")
                    for key, value in sorted_data.items():
                            # print(value)
                            return value["center"]
                    return None
        def center_robot_to_user(
                        detection_center_x: int,
                        X1_boundary: int,
                        X2_boundary: int,
                        angular_speed: float = 0.5,
                        
                ):
                    if detection_center_x < X1_boundary:
                        if self.log:
                            printclr("user to the left, adjusting.....", "blue")
                        cmd_vel_pub.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(z=angular_speed)))
                    elif detection_center_x > X2_boundary:
                        if self.log:
                            printclr("user to the right, adjusting.....", "blue")
                        cmd_vel_pub.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(z=-angular_speed)))
                    else:
                        if self.log:
                            printclr("user centered..........", "blue")
                        cmd_vel_pub.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(z=0.0)))


        """ 
        track the person_name_to_track
        """
        rospy.loginfo(f"(PersonTracker): init...")
        CAMERA_RESOLUTION = [1280,720] # width, height
        CENTER_TOLERANCE = 0.36 # percentage of the frame from the center

        # bounderies of the center, it works dont touch it
        X1_boundary = int(CAMERA_RESOLUTION[0]*0.5 - CENTER_TOLERANCE*0.5*CAMERA_RESOLUTION[0])
        X2_boundary = int(CAMERA_RESOLUTION[0]*0.5 + CENTER_TOLERANCE*0.5*CAMERA_RESOLUTION[0])


        self.centered = False # for outside checking.


        # init node for publishing to cmd_velc
        # rospy.init_node('person_tracker_nlp+cv')
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Wait for CV service
        rospy.wait_for_service('/CV_connect/req_cv')
        # CV_connect CV_connectRequest

        # create a request object
        xx = CV_srvRequest()

        # inject the type of request
        xx.cv_type.type = CV_type.TargetTracker_Detect

        # create a proxy to the service
        cv_service = rospy.ServiceProxy('/CV_connect/req_cv',CV_srv)
        
        # Embed payload into the request object and POST
        res = cv_service(xx)
        output = res.result
        # print(output)
        
            
#  rosservice call /CV_connect/req_cv "cv_type: type: 1"

        # ros log
        rospy.loginfo(f"(PersonTracker): person_name_to_track   = {self.person_name_to_track}")
        rospy.loginfo(f"(PersonTracker): closet_person_to_track = {self.closet_person_to_track}")


        while not rospy.is_shutdown() and not self.stop_tracking :
            try:
                # request to CV to track the person_name_to_track that matches the name
                res = cv_service(xx)
                # printclr(res,"blue")

                # turn the string into a dict for detections of people
                detection_list = ast.literal_eval(res.result).get('res')
                # camera_info = ast.literal_eval(res.result).get('camera_info')
                # print(camera_info)
                if self.log:
                    printclr(detection_list,"blue")

                """ returns:
                 "3": {"bbox": [507,41,1201,570],"area": 367126,"center": [553,220],"facedim": [206,158],"name": "unknown"}..... """
                
                # IF LOGGING
                # if self.response_debug:
                #     print(json.dumps(detection_list, indent=4))

                
                # for Testing
                # self.person_name_to_track = "unknown"

                
                if detection_list != {}:
                    # tracking person name match
                    if self.person_name_to_track is not None:
                    
                        #Extract the center coordinate of the center of the person that matched the name
                        name_detection_match_center = find_by_name_return_center(detection_list, self.person_name_to_track)
                        
                        if self.log:
                            # print(name_detection_match_center)
                            pass
                        
                        #Extract the X coordinate of the center
                        detection_match_center_x = name_detection_match_center[0]
                        
                    # tracking person closest to the robot
                    elif self.closet_person_to_track:
                        # ros log
                        

                        #Extract the center coordinate of the center of person closest to the robot
                        closest_detection_match_center = find_by_closest_distance_center(detection_list)

                        if self.log:
                            # closest_detection_match_center.pop("faceflatten")
                            printclr(closest_detection_match_center, "blue")
                            pass
                        
                        #Extract the X coordinate of the center
                        detection_match_center_x = closest_detection_match_center[0]
                        
                    center_robot_to_user(detection_match_center_x, X1_boundary, X2_boundary)
                else:
                    print("no detections founds")
                    # raise Exception("No detections found")
                    pass



                
            except Exception as e:
                self.centered = False
                exc_type, exc_obj, exc_tb = sys.exc_info()
                fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                printclr(e, "red")
                printclr(f"Class Error: {exc_type}", "red")
                printclr(f"Line Number: {exc_tb.tb_lineno}", "red")
                break
            
    def execute(self, userdata):

        # Log the execution stage
        rospy.loginfo(f'(IntroducePeople): Executing..')


        # Extract people from userdata
        person : Person = userdata.people_list[self.index]
        age_txt = f' {person.age}'
        if person.age == 'young':
            age_txt = 'younger than 20'
        elif person.age == 'elderly':
            age_txt = 'older than 60'
        
        
        if self.introduce_to is not None:
            introduce_to_txt = f"Hey {userdata['people_list'][self.introduce_to].name}, "
            self.person_name_to_track = userdata['people_list'][self.introduce_to].name
            self.person_tracker(continous=False)
            nlp_client.speak(text=introduce_to_txt)

        #turn to the person to be introduced to to indicate their name
        self.person_name_to_track = userdata['people_list'][person].name 
        self.person_tracker(continous=False)
        nlp_client.speak(text=f"this is {person.name}")

        #turn back to person being introduced to
        self.person_name_to_track = userdata['people_list'][self.introduce_to].name
        listener_tracker = threading.Thread(
                    target=self.person_tracker, kwargs={'continous': True})
        listener_tracker.start()
        text : str = f" their favorite drink is {person.favorite_drink}, they are {age_txt} years old, they are wearing a {person.shirt_color} shirt, and they have {person.hair_color} hair."
        nlp_client.speak(text=text)
        
        self.stop_tracking = True
        listener_tracker.join()
        return 'out1'

# Task specific state
# class DummyCv(smach.State):
#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['out1', 'out0'],
#                              output_keys=['age', 'shirt_color', 'hair_color']
#                              )

#     def execute(self, userdata):
#         userdata.age = 12
#         userdata.shirt_color = "blue"
#         userdata.hair_color = "negro"

#         return 'out1'


# Task specific state
# class CheckPeople(smach.State):
#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['out1', 'out0'],
#                              input_keys=['people_index','max_people'],
#                              output_keys=['people_index'])
        
#     def execute(self, userdata):
#         # Log the execution stage
#         rospy.loginfo(f'(CheckPeople): Executing..')

#         if userdata.people_index < userdata.max_people:
#             # Log the execution stage
#             rospy.loginfo(f'(CheckPeople): More people to add, {userdata.people_index} < {userdata.max_people}')
#             return "out1"
#         else:
#             # Log the execution stage
#             rospy.loginfo(f'(CheckPeople): Max Capacity')
#             return "out0"


def main():
    speak_debug = False
    response_debug = False
    NODE_NAME = "smach_task_receptionist"
    rospy.init_node(NODE_NAME)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out0','out1'])
    

    
    
    # Declear Variables for top-state
    sm.userdata.max_people = 2
    sm.userdata.intent = ""
    sm.userdata.people_list : list[Person] = [] 
    sm.userdata.people_index = 0
    sm.userdata.age = 0
    sm.userdata.hair_color = ""
    sm.userdata.shirt_color = ""
    sm.userdata.name = ""
    sm.userdata.favorite_drink = ""
    sm.userdata.couch_location = [0,1,2]
    timeout = 3

    host = Person(name="Game",
                    favorite_drink="milkies from your mom's titties",
                    # favorite_drink='human blood',
                    age=20,
                    shirt_color="brown",
                    hair_color="black",
                    gender='male',
                    race='Asian',
                    glasses=False)
    sm.userdata.people_list.append(host)   

    with sm:
        # TODO may have to do manual triggering of greeting
        # TODO before registering a person might have to find a way for walkie to track that person

        #------------------------Person1-------------------------------------------
        smach.StateMachine.add('GREETINGS_ASK_NAME',
                            Speak(text="""Greetings, May I have your name please?"""),
                            closet_person_to_track = True,
                            # Speak(text="Please ."),
                            transitions={'out1': 'GET_NAME',
                                            'out0': 'out0'})
        
        #  person_name_to_track = None,
        #  closet_person_to_track = True,

        
        # smach.StateMachine.add('TURN_TO_PERSON_ONE',
        #                        MoveTo(),
        #                         transitions={'out1': 'GET_NAME'})
                               
        smach.StateMachine.add('GET_NAME',
                            GetName(speak_debug=speak_debug,
                                    response_debug=response_debug,
                                    timeout=timeout),
                            transitions={'out1': 'SPEAK_ASK_OBJECT',
                                            'out0': 'out0'},
                            remapping={'listen_name': 'name'})
        
        smach.StateMachine.add('SPEAK_ASK_OBJECT',
                            Speak(text="Hello {}, What's your favorite drink?",
                            keys=["name"],
                            closet_person_to_track = True,),
                            
                            transitions={'out1': 'GET_OBJECT',
                                    'out0': 'out0'},
                            remapping={'name': 'name'})

        smach.StateMachine.add('GET_OBJECT',
                            GetObject(speak_debug=speak_debug,
                                        response_debug=response_debug,
                                        timeout=timeout),
                            transitions={'out1': 'SPEAK_RESPOND_OBJECT',
                                            'out0': 'out0'},
                            remapping={'listen_object': 'favorite_drink'})
        
        smach.StateMachine.add('SPEAK_RESPOND_OBJECT',
                            Speak(text="oh!, I like {} too! Can you stay still for a second so I can memorize your details?",
                                keys=["favorite_drink"],
                                closet_person_to_track = True,),
                            remapping={'favorite_drink': 'favorite_drink'},
                            transitions={'out1': 'IMAGE_CAPTION',
                                        'out0': 'out0'})
        
        smach.StateMachine.add('IMAGE_CAPTION',
                               ImageCaption(),
                               remapping = {'age':'age',
                                            'shirt_color':'shirt_color',
                                            'hair_color':'hair_color',
                                            'gender':'gender',
                                            'race':'race',
                                            'wearing_glasses':'wearing_glasses'},
                                transitions={'out1': 'ADD_PERSON',
                                             'undo':'out0'})
                                                
        smach.StateMachine.add('ADD_PERSON', # add person to list [0,X,0]
                                AddPerson(),
                                transitions={'out1': 'TURN_TO_COUCH', 
                                             'out0': 'out0'},
                                remapping={"age":"age",
                                        "hair_color":"hair_color",
                                        "shirt_color":"shirt_color",
                                        "name":"name",
                                        "favorite_drink":"favorite_drink",
                                        "people_list":"people_list",
                                        'people_index':'people_index'}
                                )
        
        smach.StateMachine.add('TURN_TO_COUCH',
                                 MoveTo(),
                                  transitions={'out1': 'SIT_TIGHT'})
        
        smach.StateMachine.add('SIT_TIGHT',
                               Speak(text="Please have a seat.      I am going to introduce you to our host",
                                     closet_person_to_track = True,),
                            transitions = {'out1': 'INTRODUCE_HOST',
                                            'out0':'out0'})
        
        smach.StateMachine.add('INTRODUCE_HOST',
                               IntroducePeople(people_index=0,introduce_to=1),
                               transitions = {'out1': 'INTRODUCE_PERSON_1'})

        smach.StateMachine.add('INTRODUCE_PERSON_1',
                               IntroducePeople(people_index=1,introduce_to=0),
                               transitions = {'out1': 'GREETINGS_ASK_NAME_2'})
        
        #------------------------Person2-------------------------------------------
        


        # TODO add tracking to person2
        smach.StateMachine.add('GREETINGS_ASK_NAME_2',
                            Speak(text="""Greetings, May I have your name please?"""),
                            # Speak(text="Please ."),
                            transitions={'out1': 'GET_NAME_2',
                                            'out0': 'out0'})
        
        # smach.StateMachine.add('TURN_TO_PERSON_TWO',
        #                        MoveTo(),
        #                         transitions={'out1': 'GET_NAME_2'})

        smach.StateMachine.add('GET_NAME_2',
                            GetName(speak_debug=speak_debug,
                                    response_debug=response_debug,
                                    timeout=timeout),
                            transitions={'out1': 'SPEAK_ASK_OBJECT_2',
                                            'out0': 'out0'},
                            remapping={'listen_name': 'name'})
        
        smach.StateMachine.add('SPEAK_ASK_OBJECT_2',
                    Speak(text="Hello {}, What's your favorite drink?",
                        # Speak(text="Hello {}, favorite drink?",
                            keys=["name"]),
                            transitions={'out1': 'GET_OBJECT_2',
                                    'out0': 'out0'},
                            remapping={'name': 'name'})

        smach.StateMachine.add('GET_OBJECT_2',
                            GetObject(speak_debug=speak_debug,
                                        response_debug=response_debug,
                                        timeout=timeout),
                            transitions={'out1': 'SPEAK_RESPOND_OBJECT_2',
                                            'out0': 'out0'},
                            remapping={'listen_object': 'favorite_drink'})
        
        smach.StateMachine.add('SPEAK_RESPOND_OBJECT_2',
                                Speak(text="oh!, I like {} too! Can you stay still for a second so I can memorize your details?",
                                    keys=["favorite_drink"]),
                                remapping={'favorite_drink': 'favorite_drink'},
                                transitions={'out1': 'IMAGE_CAPTION_2',
                                            'out0': 'out0'})

        smach.StateMachine.add('IMAGE_CAPTION_2',
                               ImageCaption(),
                               remapping = {'age':'age',
                                            'shirt_color':'shirt_color',
                                            'hair_color':'hair_color',
                                            'gender':'gender',
                                            'race':'race',
                                            'wearing_glasses':'wearing_glasses'},
                                transitions={'out1': 'ADD_PERSON_2',
                                             'undo':'out0' })
                                                

        smach.StateMachine.add('ADD_PERSON_2',
                                AddPerson(),
                                transitions={'out1': 'TURN_TO_COUCH_2', 
                                             'out0': 'out0'},
                                remapping={"age":"age",
                                        "hair_color":"hair_color",
                                        "shirt_color":"shirt_color",
                                        "name":"name",
                                        "favorite_drink":"favorite_drink",
                                        "people_list":"people_list",
                                        'people_index':'people_index'}
                                )

        smach.StateMachine.add('TURN_TO_COUCH_2',
                                    MoveTo(),
                                    transitions={'out1': 'SIT_TIGHT_2'})
            
        smach.StateMachine.add('SIT_TIGHT_2',
                                Speak(text="Please have a seat.     I am going to introduce you to our host"),
                                transitions = {'out1': 'INTRODUCE_HOST_2'})

        smach.StateMachine.add('INTRODUCE_HOST_2',
                               IntroducePeople(people_index=0,introduce_to=2),
                               transitions = {'out1': 'TURN_TO_HOST_2'})

        # smach.StateMachine.add('TURN_TO_HOST_2',
        #                          MoveTo(),
        #                           transitions={'out1': 'INTRODUCE_PERSON_2'})
        

        # TODO Speak to host
        smach.StateMachine.add('INTRODUCE_PERSON_2',
                               IntroducePeople(people_index=2,introduce_to=0),
                               transitions = {'out1': 'TURN_TO_PERSON_TWO_2'})
        

        #------------------------Introduct Each Other-------------------------------------------

        #TODO Speak to person2
        smach.StateMachine.add('TURN_TO_PERSON_TWO_2',
                               MoveTo(),
                                transitions={'out1': 'SPEAK_INTRODUCE_TO_EACH_OTHER'})

        smach.StateMachine.add('SPEAK_INTRODUCE_TO_EACH_OTHER',
                               Speak(text="Cool! now I am going to introduce you to each other "),
                               transitions = {'out1': 'INTRODUCE_TO_EACH_OTHER_1'})
        
        smach.StateMachine.add('INTRODUCE_TO_EACH_OTHER_1',
                               IntroducePeople(people_index=1,introduce_to=2),
                               transitions = {'out1': 'INTRODUCE_TO_EACH_OTHER_2'}) 
        
        # #TODO Speak to person1
        # smach.StateMachine.add('TURN_TO_PERSON_ONE_2',
        #                          MoveTo(),
        #                           transitions={'out1': 'INTRODUCE_TO_EACH_OTHER_2'})
        
        smach.StateMachine.add('INTRODUCE_TO_EACH_OTHER_2',
                                 IntroducePeople(people_index=2,introduce_to=1),
                                 transitions = {'out1': 'out1'})
        
        
               




    from core_nlp.emerStop import EmergencyStop
    es = EmergencyStop()
    import time
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
