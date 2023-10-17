#!/usr/bin/env python3
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

import cv2
import rospy
from geometry_msgs.msg import TwistStamped

# Define key mappings for control
key_mapping = {
    ord("w"): (1.0, 0.0),  # Forward
    ord("s"): (-1.0, 0.0),  # Backward
    ord("a"): (0.0, 1.0),  # Rotate left
    ord("d"): (0.0, -1.0),  # Rotate right
}

# Initialize the linear and angular velocity increments
d_speed = 0.1
d_angular_speed = 0.1


async def constantPublish(pub, twist):
    while not rospy.is_shutdown():
        twist.header.stamp = rospy.Time.now()
        pub.publish(twist)
        await asyncio.sleep(0.1)  # Publish rate of ~10 Hz


async def captureKeyPresses(twist):
    while not rospy.is_shutdown():
        key = cv2.waitKey(1)  # Capture key presses (wait for 1 ms)

        if key == ord(" "):
            # Stop the robot
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = 0.0
        elif key in key_mapping:
            # Update the Twist message based on the key pressed
            linear, angular = key_mapping[key]

            twist.twist.linear.x += linear * d_speed
            twist.twist.angular.z += angular * d_angular_speed

            # Ensure values stay within bounds
            twist.twist.linear.x = round(max(min(twist.twist.linear.x, 1.0), -1.0), 2)
            twist.twist.angular.z = round(max(min(twist.twist.angular.z, 1.0), -1.0), 2)
        await asyncio.sleep(0.1)


if __name__ == "__main__":
    pub = None  # Initialize the publisher outside of the try block

    try:
        # Initialize the ROS node
        rospy.init_node("amiga_vel_pub", anonymous=True)
        pub = rospy.Publisher("/amiga/cmd_vel", TwistStamped, queue_size=10)

        # Create a shared TwistStamped message
        twist = TwistStamped()
        twist.header.frame_id = "robot"

        # Create a named window for capturing key presses
        cv2.namedWindow("Robot Control", cv2.WINDOW_NORMAL)

        # Create tasks for constant publishing and key capture
        publish_task = asyncio.ensure_future(constantPublish(pub, twist))
        capture_task = asyncio.ensure_future(captureKeyPresses(twist))

        # Run the event loop with both tasks
        loop = asyncio.get_event_loop()
        tasks = asyncio.gather(publish_task, capture_task, return_exceptions=True)
        loop.run_until_complete(tasks)

        if pub is not None:
            # Stop the robot when the script is interrupted
            twist = TwistStamped()
            twist.header.frame_id = "robot"
            twist.header.stamp = rospy.Time.now()
            pub.publish(twist)

    except rospy.ROSInterruptException:
        pass

    finally:
        if pub is not None:
            # Stop the robot when the script is interrupted
            twist = TwistStamped()
            twist.header.frame_id = "robot"
            twist.header.stamp = rospy.Time.now()
            pub.publish(twist)

        cv2.destroyAllWindows()  # Close the OpenCV window
