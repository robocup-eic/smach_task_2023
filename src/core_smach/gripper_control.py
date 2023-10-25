from dy_custom.srv import SetDigitalGripper, SetDigitalGripperRequest, SetDigitalGripperResponse, SetDegreeRequest,SetDegree,SetDegreeResponse
from threading import Thread
import smach
import rospy
import nlp_client



class GripperControl(smach.State):
    def __init__(self, receive : bool = True):
        smach.State.__init__(self,outcomes=['out1'])
        self.called = False
        self.receive = receive
        # self.arm_goarm_go = walkie_cr3()
        self.client = rospy.ServiceProxy("/dy_custom/gripper/set_digital",SetDigitalGripper)
        # wait for the service to be ready
        self.client.wait_for_service()
        rospy.loginfo("(GripperControl) Gripper service is ready")
    
    def wakeword(self):
        nlp_client.ww_listen()
        rospy.loginfo("(GripperControl) WakeWord detected")
        self.called = True

    def execute(self, userdata):
        rospy.loginfo("(GripperControl) Opening gripper")
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
            if self.called:
                did_you_call_me.join()
                break
            if self.receive:
                nlp_client.speak("Once you have put your luggage in my hand, say Hey Walkie to continue")
            else:
                nlp_client.speak("Once you have taken your luggage from my hand, say Hey Walkie to continue")
        rospy.loginfo("(GripperControl) Closing gripper")
        self.client.call(SetDigitalGripperRequest(1))
        return 'out1'

