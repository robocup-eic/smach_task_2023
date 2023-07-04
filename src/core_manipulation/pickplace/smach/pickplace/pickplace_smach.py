#!/usr/bin/env python3

import rospy
from tf.transformations import quaternion_from_euler
import smach
from pickplace_include.srv import pickplace, pickplaceRequest

class Pick_from_usdata(smach.State):
    def __init__(self,side_only = False):
        smach.State.__init__(self, outcomes=['success','failed'],
                             input_keys=['pickdata'],
                             output_keys=['pickend'])
        
        self.client = rospy.ServiceProxy('pickit', pickplace)
        self.srv = pickplaceRequest()
        self.side_only = side_only

    def execute(self, userdata):
        
        self.srv = userdata.pickdata
        self.srv.check = not self.side_only
        response = self.client(self.srv)
        
        userdata.pickend = userdata.pickdata
        
        if response.graspsuccess:
            return 'success'
        else:
            return 'failed'
        
class transition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['next'], 
                             input_keys=['transin'], 
                             output_keys=['transout'])
        
    def execute(self, userdata):
        newReq = pickplaceRequest()
        
        newReq.posercam.header.frame_id = "base_link"
        newReq.posercam.header.stamp = rospy.Time.now()

        newReq.posercam.pose.position.x = 0.8
        newReq.posercam.pose.position.y = 0
        newReq.posercam.pose.position.z = 0.507

        newReq.check = False

        roll = 0
        pitch = 0
        yaw = 0
        quat = quaternion_from_euler(roll, pitch, yaw)

        newReq.posercam.pose.orientation.x = quat[0]
        newReq.posercam.pose.orientation.y = quat[1]
        newReq.posercam.pose.orientation.z = quat[2]
        newReq.posercam.pose.orientation.w = quat[3]

        newReq.dimension.x = 0
        newReq.dimension.y = 0
        newReq.dimension.z = 0
        
        userdata.transout = newReq
        return 'next'
        
class Place_from_usdata(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'], 
                             input_keys=['placedata'])
        
        self.client = rospy.ServiceProxy('placeit', pickplace)
        self.srv = pickplaceRequest()

    def execute(self, userdata):
        self.srv = userdata.placedata
        response = self.client(self.srv)
        if response.graspsuccess:
            return 'success'
        else:
            return 'failed'

def main():
    rospy.init_node('smach')
    sm = smach.StateMachine(outcomes=['pickplace_success', 'pickplace_failed'])
    
    sm.userdata.inp = pickplaceRequest()
    sm.userdata.inp.posercam.header.frame_id = "base_link"
    sm.userdata.inp.posercam.header.stamp = rospy.Time.now()

    roll = 0
    pitch = 0
    yaw = 1.3
    quat = quaternion_from_euler(roll, pitch, yaw)

    sm.userdata.inp.posercam.pose.orientation.x = quat[0]
    sm.userdata.inp.posercam.pose.orientation.y = quat[1]
    sm.userdata.inp.posercam.pose.orientation.z = quat[2]
    sm.userdata.inp.posercam.pose.orientation.w = quat[3]

    sm.userdata.inp.check = True

    sm.userdata.inp.dimension.x = 0.06
    sm.userdata.inp.dimension.y = 0.06
    sm.userdata.inp.dimension.z = 0.24

    sm.userdata.inp.posercam.pose.position.x = 0.66
    sm.userdata.inp.posercam.pose.position.y = 0
    sm.userdata.inp.posercam.pose.position.z = 0.627
    
    with sm:
        smach.StateMachine.add('Pick', Pick_from_usdata(), 
                               transitions={'success':'Transition', 'failed':'pickplace_failed'},
                               remapping={'pickdata':'inp', 'pickend':'trans'})
        
        smach.StateMachine.add('Transition', transition(), 
                               transitions={'next':'Place'}, 
                               remapping={'transin':'trans','transout':'trans'})
        
        smach.StateMachine.add('Place', Place_from_usdata(), 
                               transitions={'success':'pickplace_success', 'failed':'pickplace_failed'},
                               remapping={'placedata':'trans'})
        
    outcome = sm.execute()

if __name__ == '__main__':
    main()
