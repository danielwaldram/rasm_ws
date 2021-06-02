#!/usr/bin/env python

import copy
import rospy
import tf
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
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

        self.pub = rospy.Publisher('goal_pose_data', Pose, queue_size=10)

        rospy.init_node('move_group_python_interface_test', anonymous=True)
        self.r = rospy.Rate(0.6)
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

        # We can also print the name of teh end effector link
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
        self.last_pose = Pose()
        self.last_pose.position.x = 0
        self.last_pose.position.y = 0
        self.last_pose.position.z = 0
        self.last_pose.orientation.x = 0
        self.last_pose.orientation.y = 0
        self.last_pose.orientation.z = 0
        self.last_pose.orientation.w = 1
        self.error = [0,0,0,0,0,0]
        self.allowable_error = [0.01,0.01,0.01,0.01,0.01, 0.01]
        self.euler_current = tf.transformations.euler_from_quaternion((0, 0, 0, 1))
        self.euler_last = tf.transformations.euler_from_quaternion((0, 0, 0, 1))
        self.new_plan = True

    # The error is checked only to see if a new plan needs to be made, the face tracking node is where the goal pose is adjusted based on the screen location error
    def check_error(self, current_pose):
        self.error[0] = abs(current_pose.position.x - self.last_pose.position.x)
        self.error[1] = abs(current_pose.position.y - self.last_pose.position.y)
        self.error[2] = abs(current_pose.position.z - self.last_pose.position.z)
        self.euler_current = tf.transformations.euler_from_quaternion((current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w))
        self.euler_last = tf.transformations.euler_from_quaternion((self.last_pose.orientation.x, self.last_pose.orientation.y, self.last_pose.orientation.z, self.last_pose.orientation.w))
        self.error[3] = abs(self.euler_current[0] - self.euler_last[0])
        self.error[4] = abs(self.euler_current[1] - self.euler_last[1])
        self.error[5] = abs(self.euler_current[2] - self.euler_last[2])
        print("Error:")
        print(self.error)
        for i in range(len(self.error)):
            if self.error[i] > self.allowable_error[i]:
                self.new_plan = True



    def lookup_goal_pose(self):
        try:
            goal_from_base = self.tf_buffer.lookup_transform("base_link", "goal_pose_adj",rospy.Time(0),rospy.Duration(2.0))
            if (rospy.Time.now() - rospy.Time(goal_from_base.header.stamp.secs)) > (rospy.Duration(20)):
                return False
            return True
        except tf.LookupException:
            #If the goal pose hasn't been published yet, the RASM will go into search mode
            print("FACE POSE STILL NOT FOUND")
            return False
    def pan_up(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[4]= joint_goal[4] - pi/5
        if joint_goal[4] < -2:
            joint_goal[4] = -2
        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def pan_down(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[4] = joint_goal[4] + pi /5
        if joint_goal[4] > -0.35:
            joint_goal[4] = -0.35
        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def pan_left(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[3] = joint_goal[3] + pi / 3
        if joint_goal[3] > pi / 2:
            joint_goal[3] = pi / 2
        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def pan_right(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[3] = joint_goal[3] - 2 * pi / 3
        if joint_goal[3] < -pi / 2:
            joint_goal[3] = -pi / 2
        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def pan_right_to_center(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[3] = joint_goal[3] + pi / 3
        if joint_goal[3] > pi / 2:
            joint_goal[3] = pi / 2
        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def search_pose(self):
        self.group.set_named_target("search_pose")
        self.group.go(wait=True)
        self.group.stop()

    def screen_up_pose(self):
        self.group.set_named_target("screen_up")
        self.group.go(wait=True)
        self.group.stop()

    def shift_screen_back(self):
        screen_pose_stamped = tf2_geometry_msgs.PoseStamped()
        screen_pose_stamped.header.frame_id = "link_r_6"
        screen_pose_stamped.header.stamp = rospy.Time(0)
        screen_pose_stamped.pose.position.x = 0
        screen_pose_stamped.pose.position.y = 0
        screen_pose_stamped.pose.position.z = .25
        screen_pose_stamped.pose.orientation.x = 0
        screen_pose_stamped.pose.orientation.y = 0
        screen_pose_stamped.pose.orientation.z = 0
        screen_pose_stamped.pose.orientation.w = 1
        screen_pose_transformed = self.tf_buffer.transform(screen_pose_stamped, self.planning_frame, rospy.Duration(1))
        self.group.set_pose_target(screen_pose_transformed.pose)
        plan = self.group.plan()
        self.group.execute(plan, wait=True)
        # If the goal pose has been found the function will be exited
        self.group.stop()
        self.group.set_pose_reference_frame("base_link")

    def search_mode(self):
        #First the RASM pans down because the screen looking over someone sitting up is the most common issue
        self.pan_down()
        #RASM screen pans back up
        self.pan_up()
        # If the goal pose has been found the function will be exited
        if self.lookup_goal_pose(): return
        # First the RASM will pan left and right looking for a face
        self.pan_left()
        # If the goal pose has been found the function will be exited
        if self.lookup_goal_pose(): return
        # Panning to the other side
        self.pan_right()
        # If the goal pose has been found the function will be exited
        if self.lookup_goal_pose(): return
        self.pan_right_to_center()
        if self.lookup_goal_pose(): return

        # Move the screen back by 10 inches from the current position
        self.shift_screen_back()
        if self.lookup_goal_pose(): return

        # RASM pans right and left again
        self.pan_left()
        # If the goal pose has been found the function will be exited
        if self.lookup_goal_pose(): return
        # Panning to the other side
        self.pan_right()
        # If the goal pose has been found the function will be exited
        if self.lookup_goal_pose(): return
        self.pan_right_to_center()
        if self.lookup_goal_pose(): return

        #RASM now moves back to its search pose
        self.search_pose()
        if self.lookup_goal_pose(): return

        #The RASM will continue to pan back and forth looking for a face but the arm will no longer move
        while not self.lookup_goal_pose():
            self.pan_left()
            # If the goal pose has been found the function will be exited
            if self.lookup_goal_pose(): return
            # Panning to the other side
            self.pan_right()
            if self.lookup_goal_pose(): return
            self.pan_right_to_center()

    def eef_goal(self):
        # The transform between the rasm base and the goal pose is determined
        try:
            goal_from_base = self.tf_buffer.lookup_transform("base_link", "goal_pose_adj",rospy.Time(0),rospy.Duration(2.0))
        except tf.LookupException:
            #If the goal pose hasn't been published yet, the RASM will go into search mode
            print("FACE POSE NEVER PUBLISHED: STARTING SEARCH MODE")
            self.search_mode()
            return

        if (rospy.Time.now() - rospy.Time(goal_from_base.header.stamp.secs)) > (rospy.Duration(20)):
            print("NO TRANSFORM FOUND")
            self.search_mode()
            return


        #print("base to screen")

        #print(goal_from_base)
        my_pose = Pose()
        my_pose.position.x = goal_from_base.transform.translation.x
        my_pose.position.y = goal_from_base.transform.translation.y
        my_pose.position.z = goal_from_base.transform.translation.z
        my_pose.orientation.x = goal_from_base.transform.rotation.x
        my_pose.orientation.y = goal_from_base.transform.rotation.y
        my_pose.orientation.z = goal_from_base.transform.rotation.z
        my_pose.orientation.w = goal_from_base.transform.rotation.w
        List_of_floats = [goal_from_base.transform.translation.x, goal_from_base.transform.translation.y, goal_from_base.transform.translation.z, goal_from_base.transform.rotation.x, goal_from_base.transform.rotation.y, goal_from_base.transform.rotation.z]

        self.check_error(my_pose)
        if self.new_plan:              # if there is a new position to plan to then a now plan is set and planned to
            self.new_plan = False
            self.last_pose = my_pose   # last pose is set to the current pose only if the last pose is the one planned to
            self.group.set_pose_target(my_pose)
            plan = self.group.go(wait=False)

        #self.group.set_planner_id("RTTConnectConfigDefault")
        #self.group.set_num_planning_attempts(20)
        #self.group.set_planning_time(5)
        #raw_input("New Plan")
        self.pub.publish(my_pose)
        self.r.sleep()



def main():
    signal.signal(signal.SIGINT, signal_handler)

    tutorial = MoveGroupPythonInterfaceClass()

    tutorial.search_pose()

    try:
        while True:
            tutorial.eef_goal()
    except KeyboardInterrupt:
        print("Exited")


if __name__=='__main__':
    main()