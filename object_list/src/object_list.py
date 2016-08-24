#!/usr/bin/env python
import rospy
import numpy as np
from wasp_custom_msgs.msg import object_loc
from math import floor

april_tags = []
objects2d = []
objects3d = []

def floor_to_dec(value):
    return floor(value)

def exist_on_2d_map(object, object_list):
    for i in range(len(object_list)):
        if object.ID == object_list[i]:
            return True
    return False

def exist_on_3d_map(object, object_list):
    for i in range(len(object_list)):
        if object.ID == object_list[i][0]:
            if (floor_to_dec(object.point.x) == floor_to_dec(object_list[i][1])) and (floor_to_dec(object.point.y) == floor_to_dec(object_list[i][2])):
                return True
    return False


def new_obj_cb(data):
    global april_tags, objects2d, objects3d
    exist = False
    if (data.ID < 100):
        if exist_on_2d_map(data, april_tags) == False:
            april_tags.append(data.ID)
            print april_tags
    elif (data.ID < 200):
        if exist_on_2d_map(data, objects2d) == False:
            objects2d.append(data.ID)
            print objects2d
    else:
        if exist_on_3d_map(data, objects3d) == False:
            objects3d.append([data.ID, data.point.x, data.point.y])
            print objects3d

def shutting_down():
    global april_tags, objects2d, objects3d
    file = open('objects.txt', 'w')
    file.write("April tags \n")
    for i in range(len(april_tags)):
        file.write(april_tags[i]+"\n")
    file.write("2d objects \n")
    for i in range(len(objects2d)):
        file.write(objects2d[i]+"\n")
    file.write("3d objects \n")
    for i in range(len(objects3d)):
        file.write(str(objects3d[i][0])+" "+str(objects3d[i][1])+" "+str(objects3d[i][2])+" "+"\n")
    file.close()

# Intializes everything
def start():
    global cmd_pub, twist_cmd, linear_cmd, angular_cmd
    # publishing to "turtle1/cmd_vel" to control
    rospy.init_node('object_list')
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("/object_location", object_loc , new_obj_cb)
    rospy.on_shutdown(shutting_down)
    rospy.spin()



if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
