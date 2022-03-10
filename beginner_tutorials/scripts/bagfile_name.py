#!/usr/bin/env python
import rospy
import tf
import math
import tf2_ros
import os
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
import rosbag
import sys, signal
from os.path import expanduser
from beginner_tutorials.msg import Sensor_set_values
import time

#SET FILE PATH FOR SPECIFIC STUDY SUBJECT
home = expanduser("~")
file_path = home + "/bagfiles/user_study/3.4.22_prt2/"
files = os.listdir(file_path)
bag_name = sys.argv[1]
print(bag_name)
unnamed_files = []
for file in files:
        if "2022" in file:
                unnamed_files.append(file)
        if bag_name in file:
                print("file name already used")
                quit()

print(len(unnamed_files))
if len(unnamed_files) == 0:
        print("No files to rename")
elif len(unnamed_files) > 1:
        print("Two or more files need renamed")
elif len(unnamed_files) == 1:
        print("Renaming file")
        os.rename(file_path + unnamed_files[0],file_path + bag_name + ".bag")
