#!/usr/bin/env python
## Modification of the simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import JointState
from beginner_tutorials.msg import Sensor_set_values
import math

class node_hanlder:
    def __init__(self):
        self.yaw_joint_encoder_goal = 0
        self.roll_joint_encoder_goal = 0
        self.pitch_joint_encoder_goal = 0
        self.elbow_joint_encoder_goal = 0
        self.shoulder_joint_encoder_goal = 0
        self.height_goal = 0
        self.pub = rospy.Publisher('roll_goal', Sensor_set_values, queue_size = 10)
        self.joint_subscriber = rospy.Subscriber("joint_states", JointState, self.callback)

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position[3])
        self.yaw_joint_encoder_goal = int(data.position[3]*512/math.pi) + 352
        self.roll_joint_encoder_goal = int(-data.position[5]*512/math.pi) + 722
        self.pitch_joint_encoder_goal = int(data.position[4]*512/math.pi) + 479
        self.elbow_joint_encoder_goal = int(data.position[2]*512/math.pi) + 492
        self.shoulder_joint_encoder_goal = int(-data.position[1]*512/math.pi) + 500
        self.height_goal = int(data.position[0]*100)
        #print(self.yaw_joint)
        new_msg = Sensor_set_values()
        new_msg.sensor_values = [self.height_goal, self.shoulder_joint_encoder_goal,
                                 self.elbow_joint_encoder_goal, self.yaw_joint_encoder_goal,
                                 self.pitch_joint_encoder_goal, self.roll_joint_encoder_goal]
        print(new_msg)
        self.pub.publish(new_msg)

def talker():
    # Chatter is the name of the topic being communicated the next value is the message type
    # pub = rospy.Publisher('chatter', UInt64, queue_size=10)
    # Name of the node defined in the init
    rospy.init_node('angle_to_sens', anonymous=True)
    # I added this from the subscriber node tutorial. I don't know what topic to reference though
    # rospy.Subscriber("joint_states", JointState, callback)

    rate = rospy.Rate(1)  # 1hz
    # while not rospy.is_shutdown():
    #    hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
     #   pub.publish(hello_str)
    rate.sleep()
    node_hanlder()
    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
