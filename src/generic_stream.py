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
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfigList
from farm_ng.core.event_service_pb2 import SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file
from std_msgs.msg import String


def _farmng_to_ros_type(farmng_type):
    """Map the farmng type to the ros type."""
    # TODO: implement me
    return String


def _farmng_to_ros(farmng_msg):
    """Convert the farmng message to the ros message."""
    # TODO: implement me
    return str(farmng_msg)


async def _subscribe(client, subscribe_request, ros_publisher):
    async for event, message in client.subscribe(subscribe_request, decode=True):
        # print(f"Got reply: {message}")
        ros_msg = _farmng_to_ros(message)
        ros_publisher.publish(ros_msg)


async def subscribe(service_name, clients):
    publishers = {}

    client = clients[service_name]

    uris = await client.list_uris()

    subscribe_tasks = []
    for uri in uris:
        if uri.path not in publishers:
            topic = f"/{service_name}{uri.path}"
            print(f"Creating ROS publisher for: {topic}")
            ros_msg_type = _farmng_to_ros_type(uri.path)
            publisher = rospy.Publisher(topic, ros_msg_type, queue_size=10)

        subscribe_request = SubscribeRequest(uri=uri)
        subscribe_tasks.append(
            asyncio.create_task(_subscribe(client, subscribe_request, publisher))
        )
    return subscribe_tasks


async def run(service_config: Path) -> None:
    clients = {}
    tasks: list[asyncio.Task] = []

    # config with all the configs
    config_list: EventServiceConfigList = proto_from_json_file(
        service_config, EventServiceConfigList()
    )

    # populate the clients
    for config in config_list.configs:
        if config.port != 0:
            clients[config.name] = EventClient(config)

    for service_name in clients.keys():
        service_tasks = await subscribe(service_name, clients)
        tasks.extend(service_tasks)

    await asyncio.gather(*tasks)


if __name__ == "__main__":
    # TODO: Get the arg as required from the roslaunch file
    # parser = argparse.ArgumentParser(description='Amiga ROS Bridge')
    # parser.add_argument('--service-config', type=Path, required=True, help='Path to config file')
    # args = parser.parse_args()

    # HACK: Force the config we know is there
    service_config = (
        Path(__file__).resolve().parent.parent
        / "include"
        / "service_config_canbus.json"
    )

    try:
        # start the ros node
        loop = asyncio.get_event_loop()
        rospy.init_node("my_tutorial_node")
        rospy.loginfo("my_tutorial_node started!")
        loop.run_until_complete(run(service_config))
    except rospy.ROSInterruptException:
        pass
