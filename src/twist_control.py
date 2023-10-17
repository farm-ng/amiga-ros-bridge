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
from __future__ import annotations

import asyncio
from pathlib import Path

import rospy
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfigList
from farm_ng.core.event_service_pb2 import SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file
from farmng_ros_pipelines import create_ros_publisher
from geometry_msgs.msg import TwistStamped


def cmd_vel_callback(twist_Stamped, client: EventClient, queue: asyncio.Queue) -> None:
    twist: Twist2d = Twist2d()
    twist.linear_velocity_x = twist_Stamped.twist.linear.x
    twist.linear_velocity_y = twist_Stamped.twist.linear.y
    twist.angular_velocity = twist_Stamped.twist.angular.z
    try:
        queue.put_nowait(twist)
    except asyncio.QueueFull:
        # rospy.logwarn("Queue is full, dropping message")
        pass


async def command_task(client: EventClient, queue: asyncio.Queue) -> None:
    while True:
        twist: Twist2d = await queue.get()
        await client.request_reply("/twist", twist)


async def run(service_config: Path) -> None:
    # config with all the configs
    config_list: EventServiceConfigList = proto_from_json_file(
        service_config, EventServiceConfigList()
    )

    # populate the clients
    clients: dict[str, EventClient] = {}
    subscriptions: list[SubscribeRequest] = []

    for config in config_list.configs:
        if config.port != 0 and config.name == "canbus":
            clients[config.name] = EventClient(config)
        else:
            subscriptions = config.subscriptions

    rospy.Subscriber(
        "/amiga/cmd_vel",
        TwistStamped,
        lambda data: cmd_vel_callback(data, clients.get("canbus"), queue),
    )

    # create a publisher for the /canbus/twist stream
    tasks: list[asyncio.Task] = []

    # Queue of Twist2d messages to send to the canbus service
    queue: asyncio.Queue = asyncio.Queue(maxsize=1)

    # Set up the tasks
    for subscription in subscriptions:
        service_name = subscription.uri.query.split("=")[-1]
        path = subscription.uri.path
        if service_name == "canbus" and path == "/twist":
            tasks.append(
                asyncio.create_task(
                    create_ros_publisher(
                        clients[service_name], subscription, publish_topic="/amiga/vel"
                    )
                )
            )
            tasks.append(
                asyncio.create_task(command_task(clients[service_name], queue))
            )

    await asyncio.gather(*tasks)


if __name__ == "__main__":
    # TODO: Get the arg as required from the roslaunch file
    # parser = argparse.ArgumentParser(description='Amiga ROS Bridge')
    # parser.add_argument('--service-config', type=Path, required=True, help='Path to config file')
    # args = parser.parse_args()

    # HACK: Force the config we know is there
    service_config = (
        Path(__file__).resolve().parent.parent / "include" / "service_config.json"
    )

    try:
        # start the ros node
        loop = asyncio.get_event_loop()
        rospy.init_node("amiga_twist_control")
        rospy.loginfo("amiga_twist_control started!")
        loop.run_until_complete(run(service_config))
    except rospy.ROSInterruptException:
        pass
