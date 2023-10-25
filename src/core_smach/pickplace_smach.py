#!/usr/bin/env python3

import rospy
# from simple_environment_exporter._import import *
# from simple_environment_exporter.Simple_Environment_eXporter import Simple_Environment_eXporter as SEX
from tf.transformations import quaternion_from_euler
import smach
from pickplace_include.srv import pickplace, pickplaceRequest
import actionlib
from move_base_msgs.msg import *
from tf_manager.tf_main import transform_pose
# from simple_environment_exporter.simple_environment_exporter import SimpleEnvironmentExporter, SEX_Manipulation

class GoGo(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['out1','out0'],input_keys=['listen_intent'])

    def execute(self, ud):
        return self.exit_arena()
    
    def exit_arena(self):
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        move_base_client.wait_for_server()

        '''
        pose2: 
      position: 
        x: 4.548829078674316
        y: 2.842965841293335
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.06073186805708491
        w: 0.9981541164581234
        
        '''
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 4.548829078674316
        goal.target_pose.pose.position.y = 2.842965841293335
        goal.target_pose.pose.position.z = 0
        
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0.06073186805708491
        goal.target_pose.pose.orientation.w = 0.9981541164581234
        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()
        return 'out1'




class GoGo2(smach.State):
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
        #     pose3:
#       position: 
#         x: 4.82307243347168
#         y: 0.9959146976470947
#         z: 0.0
#       orientation: 
#         x: 0.0
#         y: 0.0
#         z: -0.9999815111330743
#         w: 0.006080903881266858

        goal.target_pose.pose.position.x = 4.82307243347168
        goal.target_pose.pose.position.y = 0.9959146976470947
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = -0.9999815111330743
        goal.target_pose.pose.orientation.w = 0.006080903881266858
        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()
        return 'out1'





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
        # def cv_res(self,name:str):
        #     self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        #     self.client = rospy.ServiceProxy("/CV_connect/req_cv",CV_srv)
        #     req = CV_srvRequest()
        #     req.cv_type.type = CV_type.BAE
        #     result:CV_srvResponse = self.client.call(req)
        #     print(result.result)
        #     res:dict = literal_eval(result.result)
        #     t_li = []

        #     try:
        #         pcl = result.pointcloud
        #         data = res.get("res")
        #         for id,value in data.items():
        #             nameo = value.get("name")
        #             if nameo == name:
        #                 x_vector = np.array(value["normal0"])
        #                 y_vector = np.array(value["normal1"])
        #                 z_vector = np.array(value["normal2"])
        #                 rotation_matrix = np.stack((x_vector,y_vector,z_vector))

        #                 cal = np.transpose(rotation_matrix)

        #                 transform_m = np.eye(4)
        #                 transform_m[:3,:3] = cal
        #                 oreintation_main = quaternion_from_matrix(transform_m)

        #                 face_z = value.get("center")
        #                 x = face_z[0]
        #                 y = face_z[1]

        #                 xyz = cv_computer().xypix_to_xyz(x,y,pcl)
        #                 if not math.isnan(xyz[0]):
        #                     t = geometry_msgs.msg.Pose()
            
                            
        #                     t.child_frame_id = "a"
        #                     t.position.x = -xyz[0]
        #                     t.position.y = -xyz[1]
        #                     t.position.z = xyz[2]

        #                     t.orientation.x = oreintation_main[0]
        #                     t.orientation.y = oreintation_main[1]
        #                     t.orientation.z = oreintation_main[2]
        #                     t.orientation.w = oreintation_main[3]
    
                        


                        

        #                 # if visible == 1:
        #                 # xyz = cv_computer().xypix_to_xyz(x,y,pcl)
        #                 # if not math.isnan(xyz[0]):
        #                 #     t = geometry_msgs.msg.TransformStamped()
        #                 #     t.header.frame_id = "computer_origin"
                            
        #                 #     t.child_frame_id = "a"
        #                 #     t.transform.translation.x = -xyz[0]
        #                 #     t.transform.translation.y = -xyz[1]
        #                 #     t.transform.translation.z = xyz[2]

        #                 #     t.transform.rotation.x = oreintation_main[0]
        #                 #     t.transform.rotation.y = oreintation_main[1]
        #                 #     t.transform.rotation.z = oreintation_main[2]
        #                 #     t.transform.rotation.w = oreintation_main[3]
        #                 #     t_li.apeend(t)
        #                 # else:
        #                 #     print("else")

        #     except KeyError:
        #         pass
            
            
        #     # main to compute origin
        #     ot = geometry_msgs.msg.TransformStamped()
        #     ot.header.stamp = rospy.Time.now()
        #     ot.header.frame_id = "zed2i_left_camera_frame"
        #     ot.child_frame_id = "computer_origin"
        #     ot.transform.translation.x = 0
        #     ot.transform.translation.y = 0
        #     ot.transform.translation.z = 0

        #     ot.transform.rotation.x = 0
        #     ot.transform.rotation.y = 0
        #     ot.transform.rotation.z = 1
        #     ot.transform.rotation.w = 0

        #     tfm = tf2_msgs.msg.TFMessage([ot]+t_li)
        #     self.pub_tf.publish(tfm)

        #     transform_pose(t,"computer_origin","zed2i_left_camera_frame")
            

        #     return transform_pose
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
        # sex = SEX(file="gg.yaml")
        # sex_man = SEX_Manipulation(sex)
        # sex_man.generate_collision_object("kitchen_table")
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
        smach.StateMachine.add('GoToTable',
                               GoGo(),
                               transitions={'out1':'Pick', 'out0':'pickplace_failed'})

        smach.StateMachine.add('Pick', Pick_from_usdata(), 
                               transitions={'success':'GoToTable2', 'failed':'pickplace_failed'},
                               remapping={'pickdata':'inp', 'pickend':'trans'})
        
        smach.StateMachine.add('GoToTable2',
                               GoGo2(),
                               transitions={'out1':'Transition', 'out0':'pickplace_failed'})      
          
        smach.StateMachine.add('Transition', transition(), 
                               transitions={'next':'Place'}, 
                               remapping={'transin':'trans','transout':'trans'})
        
        smach.StateMachine.add('Place', Place_from_usdata(), 
                               transitions={'success':'GoToTable_1', 'failed':'pickplace_failed'},
                               remapping={'placedata':'trans'})

        smach.StateMachine.add('GoToTable_1',
                               GoGo(),
                                 transitions={'out1':'Transition2', 'out0':'pickplace_failed'})
        
        smach.StateMachine.add('Transition2', transition(), 
                               transitions={'next':'Pick2'}, 
                               remapping={'transin':'trans','transout':'trans'})
        
        smach.StateMachine.add('Pick2', Pick_from_usdata(),
                               transitions={'success':'GoToTable2_1', 'failed':'pickplace_failed'},
                               remapping={'pickdata':'inp', 'pickend':'trans'})
        
        smach.StateMachine.add('GoToTable2_1',
                                 GoGo2(),
                                 transitions={'out1':'Transition3', 'out0':'pickplace_failed'})
        
        smach.StateMachine.add('Transition3', transition(),
                               transitions={'next':'Place2'},
                                remapping={'placedata':'trans'})
        
        smach.StateMachine.add('Place2', Place_from_usdata(),
                               transitions={'success':'pickplace_success', 'failed':'pickplace_failed'},
                               remapping={'placedata':'trans'})

        
    outcome = sm.execute()

if __name__ == '__main__':
    main()

#    pose2: 
#       position: 
#         x: 4.548829078674316
#         y: 2.842965841293335
#         z: 0.0
#       orientation: 
#         x: 0.0
#         y: 0.0
#         z: 0.06073186805708491
#         w: 0.9981541164581234

#     pose3:
#       position: 
#         x: 4.82307243347168
#         y: 0.9959146976470947
#         z: 0.0
#       orientation: 
#         x: 0.0
#         y: 0.0
#         z: -0.9999815111330743
#         w: 0.006080903881266858