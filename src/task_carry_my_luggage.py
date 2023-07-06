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
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import roslib
import rospy
import smach
import smach_ros
import nlp_client
import threading
from ratfin import *
from cv_connector.msg import CV_type
from cv_connector.srv import CV_srv, CV_srvRequest, CV_srvResponse
from dy_custom.srv import SetDigitalGripper, SetDigitalGripperRequest, SetDigitalGripperResponse, SetDegreeRequest,SetDegree,SetDegreeResponse
from core_nlp.utils import WakeWord , GetIntent
from core_smach.follow_person import Follow_Person
import struct
import ast
import geometry_msgs.msg
from move_base_msgs.msg import *
import actionlib

from threading import Thread


class FindPerson(smach.State):
    '''
    for when target is lost
    '''
    def __init__(self):
        smach.State.__init__(self,outcomes=['out1'])
        self.cv_client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)
        self.req = CV_srvRequest()
        self.req.cv_type.type = CV_type.HumanPoseEstimation
        
    def execute(self, userdata):
        
        def xypix_to_xyz(x, y, pointcloud):
            index = (y * pointcloud.row_step) + (x * pointcloud.point_step)
            point = struct.unpack_from('fff', pointcloud.data, offset=index)
            return point
        
        while not rospy.is_shutdown():
            res:CV_srvResponse = self.cv_client.call(self.req)
            dres:dict = ast.literal_eval(res.result)
            pcl = res.pointcloud
            result:dict  =dres.get("res")

            for id, info in result.items():
                if info.get("pose") > 0:
                    x,y = info.get("center")
                    break
            
        xyz = xypix_to_xyz(x,y,pcl)
        
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = xyz[0] -0.3
        goal.target_pose.pose.position.y = xyz[1]
        goal.target_pose.pose.position.z = 0

        yaw = math.atan2(xyz[1], xyz[0])
        quarternion_orientation = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = quarternion_orientation[0]
        goal.target_pose.pose.orientation.y = quarternion_orientation[1]
        goal.target_pose.pose.orientation.z = quarternion_orientation[2]
        goal.target_pose.pose.orientation.w = quarternion_orientation[3]

        move_base_client.send_goal(goal)
        return 'out1'

class ArmControl(smach.State):
    def __init__(self, receive : bool = True):
        smach.State.__init__(self,outcomes=['out1'])
        self.called = False
        self.receive = receive
        # self.arm_goarm_go = walkie_cr3()
        self.client = rospy.ServiceProxy("/dy_custom/gripper/set_digital",SetDigitalGripper)
    
    def wakeword(self):
        nlp_client.ww_listen()
        self.called = True

    def execute(self, userdata):
        self.client.call(SetDigitalGripperRequest(-1))
        rospy.sleep(4)
        if self.receive:
            nlp_client.speak("please put your luggage in my hand, once you are done, say Hey Walkie")
        else:
            nlp_client.speak("please take your luggage from my hand, once you are done, say Hey Walkie")
        did_you_call_me = Thread(target=self.wakeword)
        did_you_call_me.start()
        while not rospy.is_shutdown():
            if self.called:
                did_you_call_me.join()
                break
            rospy.sleep(3)
            if self.receive:
                nlp_client.speak("Once you have put your bag in my hand, say Hey Walkie to continue")
            else:
                nlp_client.speak("Once you have taken your bag from my hand, say Hey Walkie to continue")
        
        self.client.call(SetDigitalGripperRequest(1))
        return 'out1'

#------------------------DONE------------------------#

class Speak(smach.State):
    def __init__(self,text:str):
        smach.State.__init__(self, outcomes=['out1'])
        self.text = text
        

    def execute(self,userdata):
        nlp_client.speak(self.text)
        return 'out1'


class ProcessResponse(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out0','out1', 'out2'], input_keys=['listen_intent'])
        
    def execute(self, userdata):
        intent = userdata.listen_intent
        if intent == "affirm":
            return 'out1'
        elif intent == "deny":
            return 'out2'
        else:
            return 'out0'


class GoBack(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['out1','out0'],input_keys=['listen_intent'])

    def execute(self, ud):
        return self.exit_arena()
    
    def exit_arena(self):
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 0
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1

        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()
        return 'out1'
    
    
class MoveToLuggage(smach.State):
    def __init__(self):
        
        smach.State.__init__(self,outcomes=['out1'])
        

    
    def execute(self, ud):

        nlp_client.speak("please come to me")
        

        return 'out1'
    
    

def main():
    speak_debug = False
    response_debug = False
    NODE_NAME = "smach_task_receptionist"
    rospy.init_node(NODE_NAME)
    # global gricam
    # gricam = egc()
    
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out0','out1'])

    with sm:
        smach.StateMachine.add('SPEAK_ASK_FOR_HELP', 
                                Speak(text='Hello, could you please come to me with your luggage'),
                                transitions={'out1':'GET_LUGGAGE'})

        smach.StateMachine.add('GET_LUGGAGE',
                               ArmControl(receive=True),
                                transitions={'out1':'TELL_USER_FOLLOWING'})

        smach.StateMachine.add('TELL_USER_FOLLOWING',
                               Speak(text='Okay. I will now start following you. Whenever you want me to stop following, please say Hey Walkie'),
                                 transitions={'out1':'FOLLOW_USER'})
        
        smach.StateMachine.add('FOLLOW_USER',
                               Follow_Person(),
                               transitions={'out1':'ASK_IF_THERE_YET',
                                            'out2':'FIND_USER'})
        
        #TODO FIND USER IF WE LOST HIM
        smach.StateMachine.add('FIND_USER',
                                 FindPerson(),
                                 transitions={'out1':'FOLLOW_USER'})
        
        smach.StateMachine.add('ASK_IF_THERE_YET',
                               Speak(text='I heard you called me. Are we there yet?'),
                                 transitions={'out1':'GET_RESPONSE'})
        
        smach.StateMachine.add('GET_RESPONSE',
                               GetIntent(),
                               transitions={'out1':'PROCESS_RESPONSE'})
        
        smach.StateMachine.add('PROCESS_RESPONSE',
                                 ProcessResponse(),
                                 transitions={'out0':'ASK_IF_THERE_YET',
                                              'out1':'GIVE_ITEM_BACK',
                                              'out2':'TELL_USER_READY'})

        smach.StateMachine.add('TELL_USER_READY',
                               Speak(text='Okay, I am ready to follow you again. Please say Hey Walkie to continue'),
                                    transitions={'out1':'WAKEWORD_CONTINUE'})
        
        smach.StateMachine.add('WAKEWORD_CONTINUE',
                               WakeWord(),
                               transitions={'out1':'FOLLOW_USER'})

        #TODO HANDS LUGGAGE BACK, HOW TO IDK FUCK
        smach.StateMachine.add('GIVE_ITEM_BACK',
                               ArmControl(receive=False),
                               transitions={'out1':'MOVE_BACK'})
        
        smach.StateMachine.add('SPEAK_TAKE_ITEMS_BACK',
                               Speak(text='here is your luggage, please take it back. once you are done, say Hey Walkie'),
                               transitions={'out1':'WAKEWORD_END'})

        smach.StateMachine.add('WAKEWORD_END',
                               WakeWord(),
                               transitions={'out1':'MOVE_BACK'})

        # TODO MOVE BACK INTO THE ARENA,QUEING UP BEHIND PEASANTS IS OPTIONAL,
        # BUT HOW TO IDK AGAIN BRO THIS SHIT HARD
        smach.StateMachine.add('MOVE_BACK',
                               GoBack(),
                               transitions={'out1':'out0'})
        
    from core_nlp.emerStop import EmergencyStop
    es = EmergencyStop()
    # import time
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