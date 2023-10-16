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
from farm_ng.core import uri_pb2
from farm_ng.core.event_pb2 import Event
from farm_ng.core.stamp import get_stamp_by_semantics_and_clock_type
from farm_ng.core.stamp import StampSemantics
from geometry_msgs.msg import TwistStamped
from google.protobuf.any_pb2 import Any
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

# public symbols
__all__ = [
    "farmng_path_to_ros_type",
    "farmng_stamp_to_ros_time",
    "farmng_to_ros_msg",
]


def farmng_stamp_to_ros_time(event: Event) -> rospy.Time:
    """Convert a float timestamp to a ROS time.

    Args:
        event (float): The farm-ng event to extract the timestamp from.

    Returns:
        rospy.Time: The ROS time.
    """
    # unpack the stamp data
    stamp: float | None = get_stamp_by_semantics_and_clock_type(
        event, StampSemantics.DRIVER_RECEIVE, "monotonic"
    )
    if stamp is None:
        raise ValueError(
            f"Could not find DRIVER_RECEIVE timestamp for event with path: {event.uri.path}"
        )
    return rospy.Time.from_sec(stamp)


def farmng_path_to_ros_type(uri: uri_pb2.Uri):
    """Map the farmng message type to the corresponding ROS message type.

    Args:
        uri (uri_pb2.Uri): The farmng URI representing the message type.

    Returns:
        type: The ROS message type corresponding to the farmng message type.
    """

    if "canbus" in uri.query and uri.path == "/twist":
        return TwistStamped
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
    service_name: str = event.uri.query.split("=")[-1]

    # parse Twist2d message
    if event.uri.path == "/twist":
        ros_msg = TwistStamped()
        # Unpack the stamp and frame_id data
        ros_msg.header.stamp = farmng_stamp_to_ros_time(event)
        ros_msg.header.frame_id = "robot"
        # Unpack the farmng twist data
        ros_msg.twist.linear.x = farmng_msg.linear_velocity_x
        ros_msg.twist.linear.y = farmng_msg.linear_velocity_y
        ros_msg.twist.angular.z = farmng_msg.angular_velocity
        return [ros_msg]
    # parse GPS pvt message
    elif event.uri.path == "/pvt":
        ros_msg = NavSatFix()
        # Unpack the stamp and frame_id data
        ros_msg.header.stamp = farmng_stamp_to_ros_time(event)
        ros_msg.header.frame_id = "gps_antenna"
        # Unpack the GPS data
        ros_msg.latitude = farmng_msg.latitude
        ros_msg.longitude = farmng_msg.longitude
        ros_msg.altitude = farmng_msg.altitude
        return [ros_msg]
    # parse Oak IMU message
    elif event.uri.path == "/imu":
        ros_msgs = []
        for packet in farmng_msg.packets:
            ros_msg = Imu()
            # Unpack the stamp and frame_id data
            ros_msg.header.stamp = rospy.Time.from_sec(packet.gyro_packet.timestamp)
            ros_msg.header.frame_id = f"{service_name}{event.uri.path}"
            # Unpack the gyroscope data
            ros_msg.angular_velocity.x = packet.gyro_packet.gyro.x
            ros_msg.angular_velocity.y = packet.gyro_packet.gyro.y
            ros_msg.angular_velocity.z = packet.gyro_packet.gyro.z
            # Unpack the accelerometer data
            ros_msg.linear_acceleration.x = packet.accelero_packet.accelero.x
            ros_msg.linear_acceleration.y = packet.accelero_packet.accelero.y
            ros_msg.linear_acceleration.z = packet.accelero_packet.accelero.z
            ros_msgs.append(ros_msg)
        return ros_msgs
    # parse Oak Compressed Image message
    elif event.uri.path in ["/left", "/right", "/rgb", "/disparity"]:
        ros_msg = CompressedImage()
        # Unpack the stamp and frame_id data
        ros_msg.header.stamp = farmng_stamp_to_ros_time(event)
        ros_msg.header.frame_id = f"{service_name}{event.uri.path}"
        # Unpack the image data
        ros_msg.format = "jpeg"
        ros_msg.data = farmng_msg.image_data
        return [ros_msg]

    raise NotImplementedError(f"Unknown farmng message type at path: {event.uri.path}")
