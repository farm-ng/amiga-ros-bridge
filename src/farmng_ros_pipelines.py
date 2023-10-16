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
from __future__ import annotations

import rospy
from farmng_ros_conversions import farmng_path_to_ros_type
from farmng_ros_conversions import farmng_to_ros_msg

# public symbols
__all__ = [
    "subscribe",
]


async def subscribe(client, subscribe_request):
    topic = f"/{client.config.name}{subscribe_request.uri.path}"
    print(f"Creating ROS publisher for: {topic}")

    ros_msg_type = farmng_path_to_ros_type(subscribe_request.uri)
    ros_publisher = rospy.Publisher(topic, ros_msg_type, queue_size=10)

    async for event, message in client.subscribe(subscribe_request, decode=True):
        # print(f"Got reply: {message}")
        ros_msg = farmng_to_ros_msg(event.uri.path, message)
        for msg in ros_msg:
            ros_publisher.publish(msg)
