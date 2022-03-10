#!/usr/bin/env python
import rospy
import tf
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
import rosbag
import sys, signal
from os.path import expanduser
from beginner_tutorials.msg import Sensor_set_values
import time

#SET FILE PATH FOR SPECIFIC STUDY SUBJECT
home = expanduser("~")
file_path = home + "/bagfiles/1.20.22/"

bag_name = sys.argv[1]

#signal.signal(signal.SIGINT, signal_handler)

bag = rosbag.Bag(file_path + bag_name + ".bag", 'w')

def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    bag.close()
    sys.exit(0)

time_stamp = Int32()
def callback(data):
    time_stamp.data = data.data
    bag.write('time_stamp', time_stamp)

error_values = Sensor_set_values()
error_values.sensor_values = [0,0,0,0,0,0,0]
def error_callback(data):
    for i in range(len(data.sensor_values)):
        error_values.sensor_values[i] = data.sensor_values[i]
    bag.write('error_counter', error_values)



rospy.init_node('time_stamp_listener', anonymous=True)
rospy.Subscriber("/time_stamp", Int32, callback)
rospy.Subscriber("/error_counter", Sensor_set_values, error_callback)
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10)
#bag = rosbag.Bag(file_path + "test.bag", 'w')
try:
    while not rospy.is_shutdown():
        try:
            eef_from_base = tfBuffer.lookup_transform("base_link", "link_r_6", rospy.Time(0), rospy.Duration(2.0))
            bag.write('eef_from_base', eef_from_base)
            try:
                goal_from_base = tfBuffer.lookup_transform("base_link", "goal_pose", rospy.Time(0), rospy.Duration(2.0))
                bag.write('goal_from_base', goal_from_base)
                face_from_base = tfBuffer.lookup_transform("base_link", "face_pose", rospy.Time(0), rospy.Duration(2.0))
                bag.write('face_from_base', face_from_base)
                face_from_base_raw = tfBuffer.lookup_transform("base_link", "face_pose_raw", rospy.Time(0), rospy.Duration(2.0))
                bag.write('face_from_base_raw', face_from_base_raw)
            except tf.LookupException:
                pass
            rate.sleep()
        except tf.LookupException:
            pass
except KeyboardInterrupt:
    pass

print("closing bag")
bag.close()
time.sleep(0.5)
