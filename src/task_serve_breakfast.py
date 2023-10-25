'''
STAGE 1 TASK: SERVE BREAKFAST
1. Get into the kitchen
2. Fetch items (bowl, spoon, cereal box, and milk carton) and place them on the table (WE CAN CHOOSE WHETHER WE WANNA USE A TABLE OR KITCHEN ISLAND)
3. Place items onto the table

4. Pour cereal into bowl (Optional)
5. Pour milk into bowl (Optional)

6. Place spoon next to bowl
'''

import os
import signal
import roslib
import rospy
import smach
import smach_ros
import nlp_client
import threading
from ratfin import *
from core_smach.grasp_object import GraspObject
from core_smach.place_object import PlaceObject
from core_smach.move_to import Move_To
