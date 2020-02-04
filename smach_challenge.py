#!/usr/bin/env python

import rospy
import smach
from hsrb_interface import Robot
from hsrb_interface import collision_world
import hsrb_interface
import rospy
import sys
import math
import tf
import tf2_ros
import tf2_geometry_msgs
from hsrb_interface import geometry
from geometry_msgs.msg import PoseStamped, Point

##################INITIA
robot = Robot()
base = robot.try_get('omni_base')
tts = robot.try_get('default_tts')
whole_body = robot.try_get('whole_body')
gripper=robot.try_get('gripper')





# define state Initial State
class initial_state(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['outcome1'])
        
    def execute(self,userdata):
        rospy.loginfo('STATE : initial state')
        whole_body.linear_weight=20
        whole_body.move_to_go()
 

        base.go_rel(x=-.40, timeout=10)
        base.go_abs(x=0,y=0.3, timeout=10)
        return 'outcome1'
        


# define state go to hole
class go_to_table(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['outcome1','outcome2'])
        self.visited=False
    
    def execute(self,userdata):
        rospy.loginfo('State : go to table')
        whole_body.move_end_effector_pose(geometry.pose(x=-.05,z=.2,ei=3.1416,ek=1.57),'my_frame_hole')
        gripper.set_distance(.8,.5)
        if self.visited:
            return 'outcome2'
        else:
            
            self.visited = True
            return 'outcome1'
        

#class reestimate_hole 
class reestimate_hole(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['outcome1','outcome2'])
        self.visited=False
    
    def execute(self,userdata):
        rospy.loginfo('State : reestimate_hole')
        whole_body.move_end_effector_pose(geometry.pose(z=.2,ei=3.1416,ek=1.57),'chido')
        if self.visited:
            return 'outcome2'
        else:
            
            self.visited = True

            return 'outcome1'


#class ANCHOR_BODY
class anchor_body(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['outcome1'])
    
    def execute(self,userdata):
        rospy.loginfo('State : ANCHOR_BODY')
        whole_body.linear_weight=99
        whole_body.move_to_go()
        return 'outcome1'


class grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['outcome1'])
    
    def execute(self,userdata):
        rospy.loginfo('State : GRASP')
        
        gripper.set_distance(.8,1)
        whole_body.move_end_effector_pose(geometry.pose(x=-.51,y=-.51,z=.2,ei=3.1416,ek=1.57),'my_frame_hole')
        whole_body.move_end_effector_pose(geometry.pose(x=-.51,y=-.51,z=-.2,ei=3.1416,ek=1.57),'my_frame_hole')
        whole_body.move_end_effector_pose(geometry.pose(x=-.51,y=-.51,z=.2,ei=3.1416,ek=1.57),'my_frame_hole')        
        whole_body.move_to_neutral()
        base.go_rel(x=-.40, timeout=10)
   


        return 'outcome1'

                 
            
def main():
    #rospy.init_node('hsrb_challenge')
    sm=smach.StateMachine(outcomes=['END'])
    
    with sm:
        smach.StateMachine.add('INITIAL', initial_state(),
                                transitions={'outcome1':'GO_TO_TABLE'})
        smach.StateMachine.add('GO_TO_TABLE',go_to_table(), transitions={'outcome1':'ANCHOR_BODY','outcome2':'REESTIMATE_HOLE_TF'})
        smach.StateMachine.add('REESTIMATE_HOLE_TF',reestimate_hole(),transitions={'outcome1':'REESTIMATE_HOLE_TF','outcome2':'END'})
        smach.StateMachine.add('ANCHOR_BODY',anchor_body(),transitions={'outcome1':'GRASP'})
        smach.StateMachine.add('GRASP',grasp(),transitions={'outcome1':'GO_TO_TABLE'})
                        
        
        
    outcome= sm.execute()
if __name__== '__main__':
    main()