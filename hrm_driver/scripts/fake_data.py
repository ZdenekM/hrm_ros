#!/usr/bin/python

import rospy
from hrm_msgs.msg import HrmData
import random


if __name__ == "__main__":

    rospy.init_node("hrm_fake_node")

    pub = rospy.Publisher("/hrm/data", HrmData, queue_size=1)

    rate = rospy.Rate(4.0)

    while not rospy.is_shutdown():

        msg = HrmData()
        msg.timestamp = rospy.Time.now()
        msg.heart_rate = random.uniform(50, 70)
        msg.rr_interval = random.uniform(0.9, 1.1)

        pub.publish(msg)

        rate.sleep()
