
# run with conda env: nlp
import roslib
import rospy
import smach
import threading
import smach_ros
from geometry_msgs.msg import Twist, Vector3
from rospy import Publisher, init_node, Rate
from cv_connect.msg import CV_type
from cv_connect.srv import CV_connect, CV_connectRequest, CV_connectResponse

def person_tracker(person_name_to_track, continous: bool = True ):
    """ 
    track the person_name_to_track
    """

    CAMERA_RESOLUTION = [] # width, height
    CENTER_TOLERANCE = 0.36 # percentage of the frame from the center

    # bounderies of the center, it works dont touch it
    X1 = int(CAMERA_RESOLUTION[0]*0.5 - CENTER_TOLERANCE*0.5*CAMERA_RESOLUTION[0])
    X2 = int(CAMERA_RESOLUTION[0]*0.5 + CENTER_TOLERANCE*0.5*CAMERA_RESOLUTION[0])
    self.centered = False # for outside checking.

    # init node for publishing to cmd_velc
    rospy.init_node('person_tracker_nlp+cv')
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # Wait for CV service
    rospy.wait_for_service('/CV_connect/req_cv')

    cv_service = rospy.ServiceProxy('/CV_connect/req_cv')
    a = CV_connectRequest()
    a.cv_type.type = CV_type.TargetTracker_Register
    res = cv_service(a)
    print(res)

    while 1:

        # request to CV to track the person_name_to_track that matches the name

        # CV return the list of names and [X1,Y1,X2,Y2]
        cv_response = cv_track_person(person_name_to_track)
        list_of_detections = cv_response.detections

        # if the person is not detected, then don't move
        for detection in list_of_detections:
            if detection.name == person_name_to_track:
                detection_midpoint = (detection.coordinates[2] - detection.coordinates[0])/2
                # if the person is centered, break
                if detection_midpoint > X1 and detection_midpoint < X2:
                    self.centered = True
                    # stop the robot
                    cmd_vel_pub.publish(Twist())
                else:
                    # the person is not centered, move the robot
                    self.centered = False

                    # maybe add a speed curve here
                    angular_speed = 0.1

                    # if not centered, move the robot
                    if detection_midpoint < X1:
                        cmd_vel_pub.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(z=angular_speed)))
                    elif detection_midpoint > X2:
                        cmd_vel_pub.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(z=-angular_speed)))



person_tracker()