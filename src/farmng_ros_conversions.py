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

from farm_ng.core import uri_pb2
from farm_ng.core.event_pb2 import Event
from geometry_msgs.msg import Twist
from google.protobuf.any_pb2 import Any
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

# public symbols
__all__ = [
    "farmng_path_to_ros_type",
    "farmng_to_ros_msg",
]


def farmng_path_to_ros_type(uri: uri_pb2.Uri):
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


def farmng_to_ros_msg(event: Event, farmng_msg: Any) -> list:
    """Convert a farm-ng event & message to a ROS message.

    Args:
        event (Event): The event data associated with the farm-ng message.
        farmng_msg (Any): The farm-ng message to be converted, wrapped in a google.protobuf.Any message.

    Returns:
        list: A list of converted ROS messages that correspond to the farm-ng event & message.
    """
    # parse Twist2d message
    if event.uri.path == "/twist":
        ros_msg = Twist()
        ros_msg.linear.x = farmng_msg.linear_velocity_x
        ros_msg.linear.y = farmng_msg.linear_velocity_y
        ros_msg.angular.z = farmng_msg.angular_velocity
        return [ros_msg]
    # parse GPS pvt message
    elif event.uri.path == "/pvt":
        ros_msg = NavSatFix()
        ros_msg.latitude = farmng_msg.latitude
        ros_msg.longitude = farmng_msg.longitude
        ros_msg.altitude = farmng_msg.altitude
        return [ros_msg]
    # parse Oak IMU message
    elif event.uri.path == "/imu":
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
    elif event.uri.path in ["/left", "/right", "/rgb", "/disparity"]:
        ros_msg = CompressedImage()
        ros_msg.format = "jpeg"
        ros_msg.data = farmng_msg.image_data
        return [ros_msg]

    raise NotImplementedError(f"Unknown farmng message type at path: {event.uri.path}")
