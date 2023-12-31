import rospy
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import *
import actionlib
import tf2_ros
from tf2_geometry_msgs import PoseStamped
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import smach
from core_nlp.utils import Speak,WakeWord,GetIntent

class GoGo(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['out1'])
    
    def execute(self, ud):
        return super().execute(ud)
    
    def follow_person():
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        move_base_client.wait_for_server

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 2.943
        goal.target_pose.pose.position.y = -3.491
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = -0.62
        goal.target_pose.pose.orientation.w = 0.781

        rospy.sleep(10)
        

        move_base_client.send_goal(goal)
        return 'out1'


class GoBack(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['out1','out0'],input_keys=['listen_intent'])

    def execute(self, ud):
        if ud.listen_intent == 'leave_arena':
            return self.exit_arena()
        return 'out0'
    
    def exit_arena():
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        move_base_client.wait_for_server

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 2.943
        goal.target_pose.pose.position.y = -3.491
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = -0.62
        goal.target_pose.pose.orientation.w = 0.781

        rospy.sleep(10)
        

        move_base_client.send_goal(goal)
        return 'out1'

    




def main():
    sm = smach.StateMachine(outcomes=['out1','out0'])
    with sm:
        smach.StateMachine.add("GOOOOOOO",
                               GoGo(),
                               transitions={'out1':'WW'})
        smach.StateMachine.add('WW',
                               WakeWord(),
                               transitions={'out1':'COMMAND'})
        
        smach.StateMachine.add('SPEAK_ASK',
                               Speak(text='How may I help you?'),
                               transitions={'out1':'COMMAND'})
        
        smach.StateMachine.add('COMMAND',
                               GetIntent(),
                               remapping={'listen_intent':'listen_intent'})
        
        smach.StateMachine.add('GOBACK',
                               GoBack(),
                               transitions={'out1':'out1', 'out0': 'SAYTHATAGAIN'})
                               
        smach.StateMachine.add('SAYTHATAGAIN',
                               Speak('I could not understand what you said. Please say that again'),
                               transitions={'out1':'COMMAND'})
        
    sm.execute()



if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
