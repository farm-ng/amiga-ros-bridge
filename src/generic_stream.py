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
from farm_ng.core import uri_pb2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix


def _farmng_path_to_ros_type(uri: uri_pb2.Uri):
    """Map the farmng type to the ros type."""
    if "canbus" in uri.query and uri.path == "/twist":
        return Twist
    elif "gps" in uri.query and uri.path == "/pvt":
        return NavSatFix
    elif "oak" in uri.query:
        if uri.path == "/imu":
            return Imu
        elif uri.path in ["/left", "/right", "/rgb", "/disparity"]:
            return CompressedImage
    
    raise NotImplementedError(f"Unknown farmng message type: {uri}")


def _farmng_to_ros_msg(uri_path: str, farmng_msg):
    """Convert the farmng message to the ros message."""
    # parse Twist2d message
    if uri_path == "/twist":
        ros_msg = Twist()
        ros_msg.linear.x = farmng_msg.linear_velocity_x
        ros_msg.linear.y = farmng_msg.linear_velocity_y
        ros_msg.angular.z = farmng_msg.angular_velocity
        return [ros_msg]
    # parse GPS pvt message
    elif uri_path == "/pvt":
        ros_msg = NavSatFix()
        ros_msg.latitude = farmng_msg.latitude
        ros_msg.longitude = farmng_msg.longitude
        ros_msg.altitude = farmng_msg.altitude
        return [ros_msg]
    # parse Oak IMU message
    elif uri_path == "/imu":
        ros_msgs = []
        for packet in farmng_msg.packets:
            ros_msg = Imu()
            ros_msg.angular_velocity.x = packet.gyro_packet.gyro.x
            ros_msg.angular_velocity.y = packet.gyro_packet.gyro.y
            ros_msg.angular_velocity.z = packet.gyro_packet.gyro.z
            ros_msg.linear_acceleration.x = packet.accelero_packet.accelero.x
            ros_msg.linear_acceleration.y = packet.accelero_packet.accelero.y
            ros_msg.linear_acceleration.z = packet.accelero_packet.accelero.z
            ros_msgs.append(ros_msg)
        return ros_msgs
    # parse Oak Compressed Image message
    elif uri_path in ["/left", "/right", "/rgb", "/disparity"]:
        ros_msg = CompressedImage()
        ros_msg.format = "jpeg"
        ros_msg.data = farmng_msg.image_data
        return [ros_msg]

    raise NotImplementedError(f"Unknown farmng message type: {uri_path}")


async def subscribe(client, subscribe_request):
    topic = f"/{client.config.name}{subscribe_request.uri.path}"
    print(f"Creating ROS publisher for: {topic}")

    ros_msg_type = _farmng_path_to_ros_type(subscribe_request.uri)
    ros_publisher = rospy.Publisher(topic, ros_msg_type, queue_size=10)

    async for event, message in client.subscribe(subscribe_request, decode=True):
        # print(f"Got reply: {message}")
        ros_msg = _farmng_to_ros_msg(event.uri.path, message)
        for msg in ros_msg:
            ros_publisher.publish(msg)


async def run(service_config: Path) -> None:
    # config with all the configs
    config_list: EventServiceConfigList = proto_from_json_file(
        service_config, EventServiceConfigList()
    )

    # populate the clients
    clients: dict[str, EventClient] = {}
    subscriptions: list[SubscribeRequest] = []

    for config in config_list.configs:
        if config.port != 0:
            clients[config.name] = EventClient(config)
        else:
            subscriptions = config.subscriptions

    # subscribe to all the services
    tasks: list[asyncio.Task] = []

    for subscription in subscriptions:
        service_name = subscription.uri.query.split("=")[-1]
        service_tasks = asyncio.create_task(subscribe(clients[service_name], subscription))
        tasks.append(service_tasks)

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
        / "service_config.json"
    )

    try:
        # start the ros node
        loop = asyncio.get_event_loop()
        rospy.init_node("amiga_bridge_node")
        rospy.loginfo("amiga_bridge_node started!")
        loop.run_until_complete(run(service_config))
    except rospy.ROSInterruptException:
        pass
