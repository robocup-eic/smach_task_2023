#!/home/walkie/anaconda3/envs/nlp/bin/python3.9

# then chmod +x path 


# run with conda env: nlp
import roslib
import rospy
import smach
import threading
import smach_ros
import sys 
import ast 
import json 

from nlp_client import *
from ratfin import *
from geometry_msgs.msg import Twist, Vector3
from rospy import Publisher, Rate
from cv_connector.srv import CV_srv, CV_srvRequest
from cv_connector.msg import CV_type


# from cv_connector import msg, srv
# from cv_connect.msg import CV_type
# from cv_connect.srv import CV_connect, CV_connectRequest, CV_connectResponse



def prompt_user_repeat():
    speak("Sorry I didn't get that. Please rephrase that?")

# define state speak




class WakeWord(smach.State):
    """ 
    return "out1" if wakeword is detected
    >>> outcome map: {'WAKEWORD_DETECTED': 'out1'}
     
       """
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1'])

    def execute(self, userdata):
        rospy.loginfo('(WakeWord): Listening for Hey Walkie')
        ww_listen()
        rospy.loginfo('WakeWord detected')
        return "out1"


class Speak(smach.State):
    """ 
    speak hello to Person1 
    
      """
    def __init__(self, 
                 text: str, # text to speak
                 keys = None, # replace {} with userdata keys
                 person_name_to_track : str  = None, # track the listener while speaking
                 closet_person_to_track : bool = False, # track the closet person while speaking
                 response_debug :  bool = False,
                 log : bool = False):
        """  
        >>> Speak(text="Hello {}, What's your favorite drink?", keys=["name"]) 

        >>> Speak(text="Hello {}, What's your favorite drink?", 
                  keys=["name"], 
                  track_listener= "Sharon"
                  )

                            """
        if keys is None:
            keys = []

        smach.State.__init__(self, outcomes=['out1', 'out0'], input_keys=keys)

        self.response_debug = response_debug
        self.text = text
        self.log = log
        self.keys = keys
        self.person_name_to_track = person_name_to_track
        self.closet_person_to_track = closet_person_to_track
        self.stop_tracking = False
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
                    # if there's multiple isntance of the same person, return the one with the largest area
                    sorted_data = dict(sorted(detection_list.items(), key=lambda item: item[1]['area'], reverse=True))
                    if self.response_debug:
                        rospy.loginfo("sorted_data: ")
                        printclr(json.dumps(sorted_data, indent=4),"blue")
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
                    return 
        
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
                        self.centered = True
                        
                            


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
        xx.req_info = "Game"

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
                        if name_detection_match_center is not None:
                            detection_match_center_x = name_detection_match_center[0]
                        else:
                            pass
                        
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
                        if closest_detection_match_center is not None:
                            detection_match_center_x = closest_detection_match_center[0]
                        else:
                            pass
                    
                    # publish cmd_vel
                    center_robot_to_user(detection_match_center_x, X1_boundary, X2_boundary)
                    if not continous and self.centered:
                        break
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
        try:
            rospy.loginfo(f'(Speak): Executing..')
            # Prepare arguments for the format string from userdata
            args = [getattr(userdata, key)
                    for key in self.keys] if self.keys else []

            # Check if there's a person to track
            if self.person_name_to_track is not None or self.closet_person_to_track:
                # create an instance thread to continously track the person
                rospy.loginfo(f'(Speak): Adding person tracker thread..')
                listener_tracker = threading.Thread(
                    target=self.person_tracker, kwargs={'continous': True})
                listener_tracker.start()
            # Prepare the text to speak
            
            text = self.text.format(*args)

            rospy.loginfo(f'Speaking : {text}')

            # speak the intent
            speak(text)

            self.stop_tracking = True
    
            listener_tracker.join()
            # kill the listener thread
            # listener_tracker.kill()


            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"


class GetEntities(smach.State):
    """ 
    smach.StateMachine.add('GET_ENTITIES',
                           GetEntities(intent=True, name=True, object=False, location=False,
                                                       speak_debug=speak_debug,
                                                       response_debug=response_debug,
                                                       timeout=2),
                           transitions={'out1': 'NEXT_STATE',
                                        'out0': 'END'},
                           remapping={'listen_intent': 'intent',
                                      'listen_name': 'name',
                                      'listen_object': 'object',
                                      'listen_location': 'location'})
    """

    def __init__(self,
                 intent: bool = False,
                 name: bool = False,
                 object: bool = False,
                 location: bool = False,
                 confidence: bool = False,
                 speak_debug: bool = False,
                 response_debug: bool = False,
                 speak_repeat: bool = False,
                 
                 timeout=0):

        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(intent, bool):
            raise ValueError("Argument 'intent' must be of type bool")
        if not isinstance(name, bool):
            raise ValueError("Argument 'name' must be of type bool")
        if not isinstance(object, bool):
            raise ValueError("Argument 'object' must be of type bool")
        if not isinstance(location, bool):
            raise ValueError("Argument 'location' must be of type bool")
        if not isinstance(confidence, bool):
            raise ValueError("Argument 'confidence' must be of type bool")
        if not isinstance(speak_debug, bool):
            raise ValueError("Argument 'speak_debug' must be of type bool")
        if not isinstance(response_debug, bool):
            raise ValueError("Argument 'response_debug' must be of type bool")
        if not isinstance(timeout, int):
            raise ValueError("Argument 'timeout' must be of type integer")

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.speak_repeat = speak_repeat
        self.valid_out = False  # check if the output is valid

        # adding enities to extract 
        self.attributes = []

        if intent:
            self.attributes.append('intent')
        if name:
            self.attributes.append('name')
        if object:
            self.attributes.append('object')
        if location:
            self.attributes.append('location')

        # timout config
        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_' +
                                         a for a in self.attributes],
                             output_keys=['listen_' + a for a in self.attributes])

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(GetEntities): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetEntities): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # Check the object & confidence for each attribute
                for a in self.attributes:
                    attr_value = getattr(res_obj, a, "")
                    if attr_value != "" and res_obj.confidence > min_confidence:
                        setattr(userdata, 'listen_'+a, attr_value)
                        self.valid_out = True
                    else: # If any attribute is not found, break
                        self.valid_out = False
                        if self.speak_repeat: 
                            prompt_user_repeat()
                        


                # If any valid attribute is found, break
                if self.valid_out:
                    break

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetEntities): No attribute detected in the timeout period")
                raise Exception(
                    "(GetEntities): No attribute detected in the timeout period")

            return "out1"

        except Exception as e:
            printclr(e, "red")
            return "out0"


# define state GetIntent
class GetIntent(smach.State):
    """ 
    smach.StateMachine.add('GET_INTENT',
                               GetIntent(speak_debug=speak_debug,
                                         response_debug=response_debug,
                                         timeout=2),
                               transitions={'out1': 'GET_NAME',
                                            'out0': 'END'},
                               remapping={'listen_intent': 'intent'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 speak_repeat=True,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.speak_repeat = speak_repeat
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_intent', 'listen_text'],
                             output_keys=['listen_intent', 'listen_text'])

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(GetIntent): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetIntent): [{self.listen_counter}]Listening..')

                # listen for user
                # res_obj = listen(intent=True)
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.intent != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break

                # Ask the user to repeat if not valid
                if self.speak_repeat:
                    prompt_user_repeat()

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetIntent): No intent detected in the timeout period")
                raise Exception(
                    "(GetIntent): No intent detected in the timeout period")

            # TODO check if object is in the database
            # TODO if not, ask for the object
            # TODO check if it's what the user wants, check if null

            # Store intent in userdata for later use
            userdata.listen_intent = res_obj.intent

            # Store text in userdata for later use
            userdata.listen_text = res_obj.text

            # Log the intent
            rospy.loginfo(f'(GetIntent): {userdata.listen_intent}')

            # speak the intent if debug
            if self.speak_debug:
                speak(f'(GetIntent): {userdata.listen_intent}')

            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"


class GetName(smach.State):
    """ 
    smach.StateMachine.add('GET_NAME',
                               GetName(speak_debug=speak_debug,
                                       response_debug=response_debug,
                                       timeout=2),
                               transitions={'out1': 'GET_OBJECT',
                                            'out0': 'END'},
                               remapping={'listen_name': 'name'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 speak_repeat=True,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug 
        self.listen_counter = int(0)
        self.counter = 0
        self.timeout = timeout
        self.speak_repeat = speak_repeat
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             output_keys=["listen_name"])

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(GetName): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(f'(GetName): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.people != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break
                if self.speak_repeat:
                    prompt_user_repeat()

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetName): No name detected in the timeout period")
                raise Exception(
                    "(GetName): No name detected in the timeout period")

            # Store intent in userdata for later use
            name = res_obj.people
            userdata.listen_name = res_obj.people

            # Log the name
            rospy.loginfo(f'(GetName): {name}')

            # speak the name if debug
            if self.speak_debug:
                speak(f'(GetName): {name}')

            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"


class GetObject(smach.State):
    """ 
    smach.StateMachine.add('GET_OBJECT',
                               GetObject(speak_debug=speak_debug,
                                         response_debug=response_debug,
                                         timeout=2),
                               transitions={'out1': 'GET_LOCATION',
                                            'out0': 'END'},
                               remapping={'listen_object': 'object'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 speak_repeat=True,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.speak_repeat = speak_repeat
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_object'],
                             output_keys=["listen_object"])

        self.counter = 0

    def execute(self, userdata):
        try:

            # Log the execution stage
            rospy.loginfo(f'(GetObject): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetObject): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.object != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break
                if self.speak_repeat:
                    prompt_user_repeat()

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetObject): No object detected in the timeout period")
                raise Exception(
                    "(GetObject): No object detected in the timeout period")

            # Store intent in userdata for later use
            userdata.listen_object = res_obj.object

            # Log the name
            rospy.loginfo(f'(GetObject): {userdata.listen_object}')

            # speak the object if debug
            if self.speak_debug:
                speak(f'(GetObject): {userdata.listen_object}')

            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"


class GetLocation(smach.State):
    """ 
    smach.StateMachine.add('GET_LOCATION',
                               GetLocation(speak_debug=speak_debug,
                                           response_debug=response_debug,
                                           timeout=2),
                               transitions={'out1': 'END',
                                            'out0': 'END'},
                               remapping={'listen_location': 'location'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 speak_repeat=True,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.speak_repeat = speak_repeat
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_location'],
                             output_keys=["listen_location"])

        self.counter = 0

    def execute(self, userdata):
        try:

            # Log the execution stage
            rospy.loginfo(f'(GetLocation): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetLocation): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.room != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break
                if self.speak_repeat:
                    prompt_user_repeat()

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetLocation): No location detected in the timeout period")
                raise Exception(
                    "(GetLocation): No location detected in the timeout period")

            # Store intent in userdata for later use
            userdata.listen_location = res_obj.room

            # Log the name
            rospy.loginfo(f'(GetLocation): {userdata.listen_location}')

            # speak the location if debug
            if self.speak_debug:
                speak(f'(GetLocation): {userdata.listen_location}')

            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"


def main():
    speak_debug = False
    response_debug = False

    rospy.init_node('utils_nlp')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])
    sm.userdata.intent = ""
    sm.userdata.name = ""
    sm.userdata.object = ""
    sm.userdata.location = ""

    # open the container
    with sm:
        # smach.StateMachine.add('GET_ENTITIES',
        #                        GetEntities(intent=True,
        #                                    object=True,
        #                                    speak_debug=speak_debug,
        #                                    response_debug=response_debug,
        #                                    timeout=2),
        #                        transitions={'out1': 'END',
        #                                     'out0': 'END'},
        #                        remapping={'listen_intent': 'intent',
        #                                   'listen_name': 'name',
        #                                   'listen_object': 'object',
        #                                   'listen_location': 'location'})
        # smach.StateMachine.add('WAKEWORD',
        #                        WakeWord(),
        #                        transitions={'out1':'SPEAK'})

        smach.StateMachine.add('SPEAK',
                               Speak(text="Hello, what can I do for you?",
                                     person_name_to_track = "Game",
                                    #  person_name_to_track = None,
                                    #  closet_person_to_track = True,
                                     response_debug=response_debug,
                                     log=True),
                               transitions={'out1': 'END',
                                            'out0': 'END'})

        # smach.StateMachine.add('GET_INTENT',
        #                        GetIntent(speak_debug=speak_debug,
        #                                  response_debug=response_debug,
        #                                  timeout=2),
        #                        transitions={'out1': 'GET_NAME',
        #                                     'out0': 'END'},
        #                        remapping={'listen_intent': 'intent'})

        # smach.StateMachine.add('GET_NAME',
        #                        GetName(speak_debug=speak_debug,
        #                                response_debug=response_debug,
        #                                timeout=2),
        #                        transitions={'out1': 'GET_OBJECT',
        #                                     'out0': 'END'},
        #                        remapping={'listen_name': 'name'})

        # smach.StateMachine.add('GET_OBJECT',
        #                        GetObject(speak_debug=speak_debug,
        #                                  response_debug=response_debug,
        #                                  timeout=2),
        #                        transitions={'out1': 'GET_LOCATION',
        #                                     'out0': 'END'},
        #                        remapping={'listen_object': 'object'})

        # smach.StateMachine.add('GET_LOCATION',
        #                        GetLocation(speak_debug=speak_debug,
        #                                    response_debug=response_debug,
        #                                    timeout=2),
        #                        transitions={'out1': 'END',
        #                                     'out0': 'END'},
        #                        remapping={'listen_location': 'location'})

    # Execute SMACH plan
    outcome = sm.execute()
    print(sm.userdata.__dict__["_data"])


if __name__ == '__main__':
    main()
