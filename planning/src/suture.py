#!/usr/bin/env python
"""
Suture Script for Surgery Robot
Author: Leonard Weri
"""
from baxter_interface import Limb, gripper

import rospy

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, Point

from path_planner import PathPlanner
from selector import Selector

class SurgeryRobot():
    """
    Surgery Robot Class
    """
    def __init__(self, needle_length):
        """
        Constructor

        Inputs:
        needle_length: Length of needle
        """
        self._needle_length = needle_length
        
        self._suture_selector = Selector("clicked_point")
        
        self._left_planner = PathPlanner("left_arm")
        self._right_planner = PathPlanner("right_arm")

        # self._left_gripper = gripper.Gripper('left')
        # self._right_gripper = gripper.Gripper('right')

        self._left_gripper = None
        self._right_gripper = None

        # TODO set rest poses 
        self._left_rest_pose = PoseStamped()
        self._left_rest_pose.pose.position.x = 0.7
        self._left_rest_pose.pose.position.y = 0.0
        self._left_rest_pose.pose.position.z = 0.25
        self._left_rest_pose.pose.orientation.y = 1.0

        self._right_rest_pose = PoseStamped()
        self._right_rest_pose.pose.position.x = 0.692
        self._right_rest_pose.pose.position.y = 0.019
        self._right_rest_pose.pose.position.z =  0.143
        self._right_rest_pose.pose.orientation.y = 1.0

        self._left_calibrate_table_pose = PoseStamped()
        self._left_calibrate_table_pose.pose.position.x = 0.6
        self._left_calibrate_table_pose.pose.position.y = 0.0
        self._left_calibrate_table_pose.pose.position.z = 0.25
        self._left_calibrate_table_pose.pose.orientation.y = -1.0

        self._left_calibrate_camera_pose = PoseStamped()
        self._left_calibrate_camera_pose.pose.position.x = 0.659
        self._left_calibrate_camera_pose.pose.position.y = 0.246
        self._left_calibrate_camera_pose.pose.position.z = 0.158
        self._left_calibrate_camera_pose.pose.orientation.y = -1.0
    
    def move_arm(self, planner, pose):
        """
        Moves arm

        Inputs:
        planner: arm planner
        pose: goal pose (geometry_msgs/PoseStamped)
        """
        while not rospy.is_shutdown():
            try:
                goal = pose
                goal.header.frame_id = "base"

                # Might have to edit this . . . 
                plan = planner.plan_to_pose(goal, [])

                raw_input("Press <Enter> to move...")
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

    def move_to_rest(self):
        self.move_arm(self._left_planner, self._left_rest_pose)
        self.move_arm(self._right_planner, self._right_rest_pose)
    
    def calibrate_grippers(self):
        """
        Calibrate both grippers
        """
        self._left_gripper.calibrate()
        self._right_gripper.calibrate()

    def calibrate(self):
        """
        Calibrates operation area
        """
        raw_input("Starting calibration...\nPress <Enter to continue...>")
        self.move_arm(self._left_planner, self._left_calibrate_table_pose)
        self.move_arm(self._left_planner, self._left_calibrate_camera_pose)
        print("Calibration successful")

    def generate_entry_exit_points(self, entry_point):
        """
        Generates entry and exit points for suturing
        Inputs:
        entry_point: Selected entry point for suture
        Ouputs:
        Entry point for suture
        Exit point for suture
        """
        exit_point = Point()
        exit_point.x = entry_point.x
        exit_point.y = entry_point.y + (2/3) * self._needle_length
        exit_point.z = entry_point.z

        entry_point.y -= self._needle_length/3
        return entry_point, exit_point

    def suture_entry(self, entry_point):
        """
        Perform suture_entry
        Inputs:
        entry_point: point of entyr as PointStamped
        """
        # TODO: Figure out correct poses
        # Pose of entry
        entry_pose = PoseStamped()
        entry_pose.pose.position.x = entry_point.x
        entry_pose.pose.position.y = entry_point.y
        entry_pose.pose.position.z = entry_point.z
        entry_pose.pose.orientation.y = -1.0

        # Pose of insert
        insert_pose = PoseStamped()
        insert_pose.pose.position.x = entry_point.x
        insert_pose.pose.position.y = entry_point.y + self._needle_length/2
        insert_pose.pose.position.z = entry_point.z
        insert_pose.pose.orientation.y = -1.0

        # Pose of release
        release_pose = PoseStamped()
        release_pose.pose.position.x = insert_pose.pose.position.x
        release_pose.pose.position.y = insert_pose.pose.position.y
        release_pose.pose.position.z = insert_pose.pose.position.z + 0.1
        release_pose.pose.orientation.y = -1.0

        # Execute entry
        move_left_arm = lambda pose: self.move_arm(self._left_planner, pose)
        move_left_arm(self._left_rest_pose)
        move_left_arm(entry_pose)
        move_left_arm(insert_pose)
        self._left_gripper.open()
        move_left_arm(release_pose)
        move_left_arm(self._left_rest_pose)

    def suture_exit(self, grip_point):
        """
        Perform suture exit
        Inputs:
        grip_point: point to grip needle
        """
        # TODO: Figure out correct poses
        # Pose of grip
        grip_pose = PoseStamped()
        grip_pose.pose.position.x = grip_point.x
        grip_pose.pose.position.y = grip_point.y
        grip_pose.pose.position.z = grip_point.z
        grip_pose.pose.orientation.y = -1.0

        # Pose of exit
        exit_pose = PoseStamped()
        exit_pose.pose.position.x = grip_point.x
        exit_pose.pose.position.y = grip_point.y - self._needle_length/2
        exit_pose.pose.position.z = grip_point.z
        exit_pose.pose.orientation.y = -1.0

        # Pose of release
        release_pose = PoseStamped()
        release_pose.pose.position.x = self._right_rest_pose.pose.position.x
        release_pose.pose.position.y = self._right_rest_pose.pose.position.y - 0.1
        release_pose.pose.position.z = self._right_rest_pose.pose.position.z - 0.1
        release_pose.pose.orientation.y = -1.0

        # Execute exit
        move_right_arm = lambda pose: self.move_arm(self._right_planner, pose)
        move_right_arm(self._right_rest_pose)
        move_right_arm(grip_pose)
        self._right_gripper.close()
        move_right_arm(exit_pose)
        move_right_arm(release_pose)

    def suture_handoff(self):
        """
        Handoff needle from right to left gripper
        """
        self._left_gripper.close()
        self._right_gripper.open()

def main():
    rospy.init_node('moveit_node')
    robot = SurgeryRobot(0.6)
    
    # Calibrate Robot
    robot.calibrate()
    robot.calibrate_grippers()
    raw_input("Calibration succesful...\n \
               Please load patient and needle...\n \
               Press <Enter to continue... >")
    
    # Query user for input 
    robot._suture_selector.query()
    entry_points = robot._suture_selector.get_points()

    # Perform sutures
    for p in entry_points:
        entry_point, exit_point = robot.generate_entry_exit_points(p)
        robot.suture_entry(entry_point)
        robot.suture_exit(exit_point)
        robot.suture_handoff()
    robot.move_to_rest()

def test():
    rospy.init_node('moveit_node')
    robot = SurgeryRobot(0.6)

    # robot.calibrate() 
    # robot.calibrate_grippers()
    robot.move_to_rest()
    

if __name__ == '__main__':
    main()
    # test()