#!/usr/bin/env python
import rospy
import tf
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(15)

    pub = rospy.Publisher('eef_from_base', TransformStamped, queue_size = 1)
    pub2 = rospy.Publisher('goal_from_base', TransformStamped, queue_size = 1)
    pub3 = rospy.Publisher('goal_adj_from_base', TransformStamped, queue_size = 1)
    pub4 = rospy.Publisher('face_from_base', TransformStamped, queue_size=1)
    #pub4 = rospy.Publisher('eef_from_goal', TransformStamped, queue_size = 1)

    while not rospy.is_shutdown():
        try:
            eef_from_base = tfBuffer.lookup_transform("base_link", "link_r_6", rospy.Time(0), rospy.Duration(2.0))
            pub.publish(eef_from_base)
            try:
                goal_from_base = tfBuffer.lookup_transform("base_link", "goal_pose", rospy.Time(0), rospy.Duration(2.0))
                goal_adj_from_base = tfBuffer.lookup_transform("base_link", "goal_pose_adj", rospy.Time(0), rospy.Duration(2.0))
                face_from_base = tfBuffer.lookup_transform("base_link", "face_pose", rospy.Time(0), rospy.Duration(2.0))
                pub2.publish(goal_from_base)
                pub3.publish(goal_adj_from_base)
                pub4.publish(face_from_base)
            except tf.LookupException:
                pass
            rate.sleep()
        except tf.LookupException:
            pass