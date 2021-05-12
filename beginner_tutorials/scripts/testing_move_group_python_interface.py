#!/usr/bin/env python

import copy
import rospy
import tf
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
import moveit_msgs.msg
#import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
import sys, signal

def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)


class MoveGroupPythonInterfaceClass(object):
    def __init__(self):
        super(MoveGroupPythonInterfaceClass, self).__init__()
        #First initializing moveit commander and a rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_test', anonymous=True)
        self.r = rospy.Rate(.1)
        #Instantiate a robotcommander object. This object is the outer-level interface to the robot:
        robot = moveit_commander.RobotCommander()
        #Instantiate a PlanningScene interface object. This object is an interface to the world surrounding the robot
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a movegroupcommander object. This object is an interface to one group of joints.
        group_name="full_rasm"
        group = moveit_commander.MoveGroupCommander(group_name)

        # A displaytrajectory publisher can be used to publish trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # We can get the name of teh reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "===================== Reference frame: %s" % planning_frame

        # We can also print the name of the end effector link
        eef_link = group.get_end_effector_link()
        print "===================== End effector: %s" % eef_link

        #Printing the entire state of the robot
        print "===================== Printing robot state"
        print robot.get_current_state().joint_state.position[3]

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Assigning variables
        self.robot = robot
        self.scene = scene
        self.group = group

        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link

    def offset_pose(self):
        #self.group.set_pose_reference_frame("link_r_6")
        #print "===================== Reference frame: %s" % self.group.get_pose_reference_frame()
        screen_pose = self.group.get_current_pose().pose
        screen_pose.position.x = -.1
        screen_pose.position.y = 0
        screen_pose.position.z = 0
        screen_pose.orientation.x = 0
        screen_pose.orientation.y = 0
        screen_pose.orientation.z = 0
        screen_pose.orientation.w = 1

        screen_pose_stamped = tf2_geometry_msgs.PoseStamped()
        screen_pose_stamped.pose = screen_pose
        screen_pose_stamped.header.frame_id = "link_r_6"
        screen_pose_stamped.header.stamp = rospy.Time(0)

        screen_pose_transformed = self.tf_buffer.transform(screen_pose_stamped, self.planning_frame, rospy.Duration(1))
        self.group.set_pose_target(screen_pose_transformed.pose)
        plan = self.group.plan()
        print(screen_pose)
        raw_input("Planning Complete")
        self.group.execute(plan, wait=True)
        # If the goal pose has been found the function will be exited
        self.group.stop()
        self.group.set_pose_reference_frame("base_link")

    def eef_goal(self):
        self.group.set_pose_target(my_pose)
        self.group.set_planner_id("RTTConnectConfigDefault")
        self.group.set_num_planning_attempts(20)
        self.group.set_planning_time(5)
        #plan = self.group.go(wait=True)
        #raw_input("New Plan")
        self.r.sleep()

    def interrupt_motion_plan(self):
        self.group.set_named_target("search_pose")
        self.group.go(wait = True)
        self.group.set_named_target("screen_up")
        self.group.go(wait = False)



def main():
    signal.signal(signal.SIGINT, signal_handler)

    tutorial = MoveGroupPythonInterfaceClass()


    try:
        tutorial.interrupt_motion_plan()
    except KeyboardInterrupt:
        print("Exited")


if __name__=='__main__':
    main()