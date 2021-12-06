#!/usr/bin/env python
"""
Selector Class for Surgery Robot
Author: Leonard Wei
"""
import rospy
from geometry_msgs.msg import PointStamped

class Selector():
    """
    Point Clould Selector for Surgery Robot

    Extracts points from point cloud when published by rviz's Publish Point function
    """
    def __init__(self):
        """
        Constructor
        """
        self._sub = None
        self._total = 0
        self._points = []

    def clear(self):
        """
        Clears Selector of points and resets total
        """
        self._total = 0
        self._points = []

    def get_points(self):
        return self._points

    def callback(self, message):
        self._points.append(message.point)
        print("Entry {}:\n{}".format(len(self._points), message.point))

    def query(self):
        """
        Makes a querys
        """
        self._total = int(raw_input("Please enter the number of sutures you would like to perform...\n"))
        self._sub = rospy.Subscriber("clicked_point", PointStamped, self.callback)
        while(len(self._points) < self._total): continue
        self._sub.unregister()
        self._sub = None
        print("Collected {} points!".format(len(self._points)))

def main():
    rospy.init_node("selection")
    test = Selector()
    test.query()
    print(test.get_points())
    test.clear()
    print("Selector cleared!")
    test.query()

if __name__ == '__main__':
    main()