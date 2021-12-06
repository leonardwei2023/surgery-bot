#!/usr/bin/env python
"""
Selection Script for Surgery Robot
Author: Leonard Wei
"""
import rospy
from geometry_msgs.msg import PointStamped

suture_total, suture_index = 0, 1
sutures = []
sub = None

def callback(message):
    global suture_index, sutures
    sutures.append(message.point)
    print("Entry {}:\n{}".format(suture_index, message.point))
    suture_index += 1

def query():
    global suture_total, sub
    suture_total = int(raw_input("Please enter the number of sutures you would like to perform...\n"))
    sub = rospy.Subscriber("clicked_point", PointStamped, callback)
    while(suture_index <= suture_total): continue
    sub.unregister()
    print("Collected {} points!".format(len(sutures)))

def main():
    query()

if __name__ == '__main__':
    rospy.init_node("selection")
    main()