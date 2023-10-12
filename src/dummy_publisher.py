#!/usr/bin/env python
# Copyright (c) farm-ng, inc.
#
# Licensed under the Amiga Development Kit License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://github.com/farm-ng/amiga-dev-kit/blob/main/LICENSE
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import asyncio

import rospy
from std_msgs.msg import String


async def publisher1():
    pub = rospy.Publisher("dummy_topic1", String, queue_size=10)
    # rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        dummy_message = "Publisher 1: This is a dummy message"
        rospy.loginfo(dummy_message)
        pub.publish(dummy_message)
        await asyncio.sleep(1)  # 1 Hz


async def publisher2():
    pub = rospy.Publisher("dummy_topic2", String, queue_size=10)
    # rate = rospy.Rate(2)  # 2 Hz
    while not rospy.is_shutdown():
        dummy_message = "Publisher 2: This is another dummy message"
        rospy.loginfo(dummy_message)
        pub.publish(dummy_message)
        await asyncio.sleep(2)  # 2 Hz


if __name__ == "__main__":
    try:
        rospy.init_node("dummy_publishers", anonymous=True)
        loop = asyncio.get_event_loop()
        tasks = [loop.create_task(publisher1()), loop.create_task(publisher2())]
        loop.run_until_complete(asyncio.gather(*tasks))
    except rospy.ROSInterruptException:
        pass
