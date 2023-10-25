#!/usr/bin/python

import yaml
from geometry_msgs.msg import Pose
class read_yaml :
    def __init__(self) :
        self.f = open('/home/walkie/smach_task_2023/src/station_pos.yaml')
        self.docs = yaml.load_all(self.f, Loader=yaml.FullLoader)
        self.pose_list = []

    def find(self, find_type, room_name) :
        self.pose_list.clear()
        ret = []
        pos = Pose()
        for doc in self.docs :
            if(find_type in doc) :
                dict_room = doc[find_type]
                list_pos_key = dict_room[room_name].keys()
                for pose in list_pos_key :
                    self.pose_list.append(dict_room[room_name][pose])
        for pose in self.pose_list :
            pos.position.x = pose["position"]["x"]
            pos.position.y = pose["position"]["y"]
            pos.position.z = pose["position"]["z"]
            pos.orientation.x = pose["orientation"]["x"]
            pos.orientation.y = pose["orientation"]["y"]
            pos.orientation.z = pose["orientation"]["z"]
            pos.orientation.w = pose["orientation"]["w"]
            ret.append(pos)
        return ret

if __name__ == "__main__" :
    ry = read_yaml()
    for pose in ry.find("find_person", "living_room") :
        print()
        print(pose)
    

    # for pose in ry.find("find_object", "living_room") :
    #     print(pose["position"])
    #     print(pose["orientation"])