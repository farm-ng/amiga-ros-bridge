#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import asyncio

async def publisher1():
    pub = rospy.Publisher('dummy_topic1', String, queue_size=10)
    # rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        dummy_message = "Publisher 1: This is a dummy message"
        rospy.loginfo(dummy_message)
        pub.publish(dummy_message)
        await asyncio.sleep(1) # 1 Hz

async def publisher2():
    pub = rospy.Publisher('dummy_topic2', String, queue_size=10)
    # rate = rospy.Rate(2)  # 2 Hz
    while not rospy.is_shutdown():
        dummy_message = "Publisher 2: This is another dummy message"
        rospy.loginfo(dummy_message)
        pub.publish(dummy_message)
        await asyncio.sleep(2) # 2 Hz

if __name__ == '__main__':
    try:
        rospy.init_node('dummy_publishers', anonymous=True)
        loop = asyncio.get_event_loop()
        tasks = [loop.create_task(publisher1()), loop.create_task(publisher2())]
        loop.run_until_complete(asyncio.gather(*tasks))
    except rospy.ROSInterruptException:
        pass
