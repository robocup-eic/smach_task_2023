
""" 

Guidelines for writing a state:

Consult example_smach_state.py for an ideal state
--------------------------------------------
--MUST HAVE: 

1. OUTCOME INFORMATION: Have to declear all
    outcomes=['out1','out2','loop','undo','fail'] # out2 is optional, a good practice should have loop & undo

2. INPUT/OUTPUT DATA: Have to declare all
    input_keys=['data1', 'data2'],
    output_keys=['data1', 'data2']

2. LOGGING: EACH STEP MUST HAVE LOGGING TO ROSPY
    rospy.loginfo('Executing state State1')
    rospy.loginfo(f'({name of class state}}): Executing..')
    rospy.loginfo(f'(AddPerson): {p.name} added to people_list. {p.__dict__}')

3. REMAPPING: ADD TO LIST FOR CONSTRUCTOR 
    # Will be added to the state machine by inputs and outputs
    remappings = {'data1': 'data1', 'data2': 'data2'}  # Use actual remappings

4. EXCEPTION HANDLING: return loop if exception or undo

5. FIX DATATYPE: NO DYNAMIC TYPE
    # Fix datatype to string, int, float, bool, list, dict
    log : bool = False # have default value
    # Raise exceptions if any entity parameter is not of type bool
    if not isinstance(intent, bool):
        raise ValueError("Argument 'intent' must be of type bool")
    if None then raise exception for variable CANNOT BE NONE

--------------------------------------------
--OPTIONAL:  

"""

import rospy
import smach
import nlp_client
import threading
from ratfin import *
import socket
import cv2
try:
    from custom_socket import CustomSocket
except:
    from core_cv.custom_socket import CustomSocket # when running from main
import os 
import os
import sys      
import time
import json
import ast
sys.path.append("smach_task_2023/core_nlp/") # specific to cv folder
# from emerStop import EmergencyStop, StopEnd

from geometry_msgs.msg import Twist, Vector3
from rospy import Publisher, Rate
from cv_connector.srv import CV_srv, CV_srvRequest
from cv_connector.msg import CV_type

os.environ["QT_LOGGING_RULES"] = "*=false"

# # Model Smach States
class RegisterFace(smach.State):

    def __init__(self, 
                 log : bool = False,
                 nlp : bool = True,
                 person_tracking : bool = True,
                 timeout_tries: int = 0 # 0 means infinite tries
                 ):
        """ 
        input_keys=['name'],
        output_keys=['name']
        TemplateVersion 1.1.0 
        """
        self.log = log
        self.nlp = nlp
        self.person_tracking = person_tracking
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        if not isinstance(nlp, bool):
            raise ValueError("Argument 'nlp' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1'],
                            #  outcomes=['out1','out2','loop','undo','timeout'],
                             input_keys=['name'],
                             output_keys=['name'])
        
        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries

    def person_tracker(self, 
                    #    userdata, 
                       continous: bool = True ):
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
                            self.centered = True

        rospy.loginfo(f"(PersonTracker): init...")
        CAMERA_RESOLUTION = [1280,720] # width, height
        CENTER_TOLERANCE = 0.36 # percentage of the frame from the center

        # bounderies of the center, it works dont touch it
        X1_boundary = int(CAMERA_RESOLUTION[0]*0.5 - CENTER_TOLERANCE*0.5*CAMERA_RESOLUTION[0])
        X2_boundary = int(CAMERA_RESOLUTION[0]*0.5 + CENTER_TOLERANCE*0.5*CAMERA_RESOLUTION[0])


        self.centered = False # for outside checking.


        # init node for publishing to cmd_vel
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Wait for CV service
        rospy.wait_for_service('/CV_connect/req_cv')

        # create a request object
        payload = CV_srvRequest()

        # inject the type of request
        payload.cv_type.type = CV_type.TargetTracker_Detect

        # create a proxy to the service
        cv_service = rospy.ServiceProxy('/CV_connect/req_cv',CV_srv)

        while not rospy.is_shutdown() and not self.stop_tracking :
            try:
                # request to CV to track the person_name_to_track that matches the name
                res = cv_service(payload)
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
                    if self.person_tracking:
                        # ros log
                    
                        #Extract the center coordinate of the center of person closest to the robot
                        closest_detection_match_center = find_by_closest_distance_center(detection_list)

                        if self.log:
                            # closest_detection_match_center.pop("faceflatten")
                            printclr(closest_detection_match_center, "blue")
                            pass
                        
                        #Extract the X coordinate of the center
                        detection_match_center_x = closest_detection_match_center[0]
                    
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
            # Log the execution stage
            rospy.loginfo(f'(RegisterFace): Executing..')

            
            # Do something
            if userdata.name == "":
                raise Exception("No name given")
            
            # print(userdata)

            # Check if there's a person to track
            if self.person_tracking:
                # create an instance thread to continously track the person
                rospy.loginfo(f'(Speak): Adding person tracker thread..')
                listener_tracker = threading.Thread(
                    target=self.person_tracker, kwargs={'continous': True})
                listener_tracker.start()
            # Prepare the text to speak
            ## Executing the registering face
            # Userdata verification
            rospy.loginfo(f'(RegisterFace): using userdata.name: {userdata.name}.')

            # inject the type of request

            # Wait for CV service
            rospy.wait_for_service('/CV_connect/req_cv')
            
            # create a proxy to the service
            cv_service_register = rospy.ServiceProxy('/CV_connect/req_cv',CV_srv)



            ## Executing the registering face


            # create a request object with the name of the person to register
            register_payload = CV_srvRequest()
            register_payload.cv_type.type = CV_type.TargetTracker_Register
            register_payload.req_info = userdata.name
            
            ## Finishing the registering face
            # send a flag to end the thread
            self.stop_tracking = True
            listener_tracker.join()

            if self.nlp:
                time.sleep(1.5)
                rospy.loginfo(f'(Speak): capturing in 3. 2. 1.')
                nlp_client.speak(f"capturing in 3. 2. 1.")
            rospy.loginfo(f'(RegisterFace): capturing...')
            # send the request to CV to register the face
            cv_service_register(register_payload)
            
            rospy.loginfo(f'(RegisterFace): SUCCESS,  {userdata.name} registered')
            if self.nlp:
                nlp_client.speak(f"{userdata.name} face registered")
            
            if self.nlp:
                time.sleep(1.5)
                nlp_client.speak(f"captured complete")
                    
                    
            
            rospy.loginfo(f'(RegisterFace): exiting...')
            

            return "out1"
        except Exception as e:
            
            self.centered = False
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            printclr(e, "red")
            printclr(f"Class Error: {exc_type}", "red")
            printclr(f"Line Number: {exc_tb.tb_lineno}", "red")
            return "undo" 


# Model Smach States
class DetectFace(smach.State):
    """ 
    TemplateVersion 1.1.0 
    """
    def __init__(self, 
                 log : bool = False,
                 timeout_tries: int = 0 # 0 means infinite tries
                 ):
        self.log = log
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','undo'],
                            #  outcomes=['out1','out2','loop','undo','timeout'],
                             input_keys=['name'],
                             output_keys=['name'])
        
        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries


    def execute(self, userdata):
        try:
            # Receive and return list of names

            # Log the execution stage
            rospy.loginfo(f'(DetectFace): Camera init...')

            ## Setup cap 
            cap = cv2.VideoCapture(0)
            cap.set(4, 480)
            cap.set(3, 640)

            # Setup socket
            host = socket.gethostname()
            port = 12304
            c = CustomSocket(host, port)
            c.clientConnect()

            # Log the execution stage
            rospy.loginfo(f'(DetectFace): Entering Loop..')
            while cap.isOpened():
                # Read from Camera
                ret, frame = cap.read()
                cv2.imshow("client_cam", frame)
                
                # Post to Socket
                res = c.detect(frame)
                # {'name': 'Game'}
                # if detect a known person

                #TODO: Add the ability to add the person to be found 
                if res != {}:

                    name = res["name"]
                    # Log the execution stage
                    rospy.loginfo(f'(DetectFace): Found a person named {name}')
                    if self.nlp:
                        nlp_client.speak(f"Found {name}")
                    if self.log:
                            print(res)

                    cap.release()
                    # Log the execution stage
                    rospy.loginfo(f'(RegisterFace): exiting...')
                    
                    return "out1"

                key = cv2.waitKey(1)
                if key == ord("q"):
                    cap.release()
                    return "out1"

            cv2.destroyAllWindows()
        except Exception as e:
            print(e)
            cap.release()
            return "undo"

def main():
    log = False
    # Initialize the node
    NODE_NAME = "smach_cv_face_recognition"
    rospy.init_node(NODE_NAME)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out1'])

    # Declear Variables for top-state
    sm.userdata.name = "Game"

    with sm:
        smach.StateMachine.add('REGISTER_FACE',
                            RegisterFace(log=log),
                            remapping={'name': 'name'},
                            transitions={'out1': 'out1',
                                         'undo': 'out1',}
                                        )
        # smach.StateMachine.add('DETECT_FACE',
        #                     DetectFace(log=log),
        #                     remapping={'name': 'name'},
        #                     transitions={'out1': 'out1',
        #                                  'undo': 'out1',}
        #                                 )
        # smach.StateMachine.add('STOP_END',
        #                     StopEnd(),
        #                     remapping={'stop': 'stop'},
        #                     transitions={'out1': 'out1'}
        #                                 )

    # Create a thread to execute the smach container
    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    


    # es = EmergencyStop()
    # es_thread = threading.Thread(target=es.execute)
    # es_thread.start()
    # es.emer_stop_handler(sm.userdata)

    
if __name__ == '__main__':
    main()




