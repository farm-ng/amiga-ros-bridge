# amiga-ros-bridge

## Overview

The ROS bridge, currently supported for ROS Noetic, interfaces with the
[Amiga gRPC services](https://github.com/farm-ng/farm-ng-amiga) to:

- Stream data from the Amiga brain services
- Control the Amiga using the available vehicle control APIs.

Using the ROS bridge with the amiga requires an Amiga OS `>= v2.0.0`.

> For Amiga brains running Amiga OS `1.0` - `1.3`, please refer to
> [github.com/farm-ng/amiga-ros-bridge-v1](https://github.com/farm-ng/amiga-ros-bridge-v1).

### Usage

The recommended usage of the `amiga-ros-bridge`, and instructions on this page,
reflect the recommended user workflow of running the `amiga-ros-bridge` on a development
PC with a remote `gRPC` connection to the Amiga brain (over wifi).

The connection can be configured by changing the `host` fields in [`include/service_config.json`](/include/service_config.json) from `localhost` to your robot's name (e.g., `element-vegetable`).

> You may refer to the [`amiga-ros-bridge` Docker Setup](https://github.com/farm-ng/amiga-ros-bridge/blob/main/docker_setup.md)
> for **experimental** instructions on running the `amiga-ros-bridge` directly on the brain,
> inside of a docker container.
> However, please not that these are experimental and do not reflect the recommended workflow.

### Default Topics

#### `amiga_streams`

- **`/canbus/twist`**
  - Published by the `amiga_streams` node
  - Type: [`geometry_msgs/TwistStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)
  - Corresponds to [`Twist2d`](https://github.com/farm-ng/farm-ng-amiga/blob/main/protos/farm_ng/canbus/canbus.proto) proto message from the canbus service.

- **`/filter/state`**
  - Published by the `amiga_streams` node
  - Type: [`nav_msgs/Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
  - Corresponds to [`FilterState`](https://github.com/farm-ng/farm-ng-amiga/blob/main/protos/farm_ng/filter/filter.proto) proto message from the state estimation filter service.

- **`/gps/pvt`**
  - Published by the `amiga_streams` node
  - Type: [`sensor_msgs/NavSatFix`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html)
  - Corresponds to [`GpsFrame`](https://github.com/farm-ng/farm-ng-amiga/blob/main/protos/farm_ng/gps/gps.proto) proto message from the gps service.

- **`/oak0/imu`**
  - Published by the `amiga_streams` node
  - Type: [`sensor_msgs/Imu`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)
  - Corresponds to [`OakImuPacket`](https://github.com/farm-ng/farm-ng-amiga/blob/main-v2/protos/farm_ng/oak/oak.proto) proto message from the oak camera service.

- **`/oak0/left`**
  - Published by the `amiga_streams` node
  - Type: [`sensor_msgs/CompressedImage`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CompressedImage.html)
  - Corresponds to [`OakFrame`](https://github.com/farm-ng/farm-ng-amiga/blob/main-v2/protos/farm_ng/oak/oak.proto) proto message from the oak camera service.

> TIP: You can check for any additional streams with
>
> ```bash
> $ rostopic list
>
> # And you should see:
> #
> # /canbus/twist
> # /filter/state
> # /gps/pvt
> # /oak0/imu
> # /oak0/left
> # /rosout
> # /rosout_agg
> ```
>
> And inspect the topic with:
>
> ```bash
> $ rostopic info /canbus/twist
>
> # And you should see:
> #
> # Type: geometry_msgs/TwistStamped
> #
> # Publishers:
> #  * /amiga_streams (http://<your-pc>:<port>)
> #
> # Subscribers: None
> ```

#### `twist_control`

- **`/amiga/vel`**
  - Published by the `twist_control` node
  - Type: [`geometry_msgs/TwistStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)
  - Corresponds to [`Twist2d`](https://github.com/farm-ng/farm-ng-amiga/blob/main/protos/farm_ng/canbus/canbus.proto) proto message

- **`/amiga/cmd_vel`**
  - Subscribed by the `twist_control` node
  - Type: [`geometry_msgs/TwistStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)
  - Corresponds to [`Twist2d`](https://github.com/farm-ng/farm-ng-amiga/blob/main/protos/farm_ng/canbus/canbus.proto) proto message

## Setup

These instructions are for installing the `amiga-ros-bridge` to interact with
Amiga brains running the **Amiga OS 2.0** through a ROS bridge.

> For Amiga brains running Amiga OS `1.0` - `1.3`, please refer to
> [github.com/farm-ng/amiga-ros-bridge-v1](https://github.com/farm-ng/amiga-ros-bridge-v1).

### Install dependencies

To run the `amiga_ros_bridge` on your PC, you will need to have already followed the
[ROS Noetic install instructions](http://wiki.ros.org/noetic/Installation/Ubuntu)

> NOTE: Using ROS on your PC means you're either running some flavor of Linux or know how to run ROS in a VM on your other OS.

### Setup a `catkin_ws`

> More detail is available at [ROS - Create a catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace).

```bash
# You can choose a different name for `catkin_ws` here
# But the remaining documentation assumes this is what you've used
mkdir -p ~/catkin_ws/src
```

### Clone the amiga-ros-bridge

```bash
# Navigate to the src/ directory of your catkin_ws
cd ~/catkin_ws/src/
# Clone the amiga-ros-bridge, with submodules
git clone --recursive https://github.com/farm-ng/amiga-ros-bridge.git
```

### Build your catkin workspace

```bash
# Navigate to the catkin_ws
cd ~/catkin_ws/

# Run catkin_make
catkin_make
```

### Build the venv

```bash
# Navigate to the catkin_ws
cd ~/catkin_ws/

# Build the venv
./src/amiga-ros-bridge/setup_venv.sh
# You will be prompted for your password
```

### Add the `venv` to `devel/setup.bash`

```bash
# Navigate to the catkin_ws
cd ~/catkin_ws/

# Get the current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Add sourcing the amiga-ros-bridge to devel/setup.bash
echo "source $DIR/src/amiga-ros-bridge/source_venv.sh" >> devel/setup.bash
```

Now when you `source devel/setup.bash`,
the `amiga-ros-bridge/venv/` will also be sourced.

## Use the ros bridge

### Amiga streams

You can use the `amiga_streams` node to stream sensor data from the Amiga brain services.
Each stream topic will be setup as a ROS publisher,
and the farm-ng proto message will be converted to a corresponding ROS message.

To run the `amiga_streams` bridge:

Configure the connection by changing the `host` fields in
[`include/service_config.json`](/include/service_config.json) from `localhost`
to your robot's name (e.g., `element-vegetable`).

Then run:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch amiga_ros_bridge amiga_streams.launch
```

You can subscribe to published values from the amiga with ROS command line tools.
For example:

```bash
source ~/catkin_ws/devel/setup.bash
rostopic echo /canbus/twist
```

#### Modify the streams

You can edit the topics streamed by modifying the `subscriptions` field in
[`include/service_config.json`](/include/service_config.json).

For instance, you could stream the `/rgb` image stream from `oak0` instead of the
`/left` stereo camera by changing `path` of the subscription.

Or you could add an `oak1` client and create an additional stream of the `/oak1/imu`.

Do this by adding to the configs, e.g.:

```json
{
    "name": "oak1",
    "port": 50011,
    "host": "localhost"
}
```

And adding a subscription:

```json
{
    "uri": {
        "path": "/imu",
        "query": "service_name=oak1"
    },
    "every_n": 1
}
```

It is recommended to adjust the [`include/service_config.json`](/include/service_config.json)
to suit your needs.
Add the streams that you need and remove any you do not to save bandwidth and computational resourced.

> NOTE: A farm-ng -> ROS conversion for all messages has not yet been implemented.
>
> If the message/topic you need is not yet supported, please either:
>
> - (good) Post a request in [discourse.farm-ng.com](https://discourse.farm-ng.com/)
> - (better) Open an issue in this repository with your feature request
> - (best) Contribute the new stream by opening a PR to this open source repository!

### Twist control

> WARNING: When the dashboard is in auto mode, this will cause the Amiga to drive.
> Make sure the area is clear before using this.
>
> You can also test this by sending the commands when the Amiga dashboard
> is not in AUTO READY or AUTO ACTIVE and see the commands being sent with
> the red needle on the auto page.

You can use the `twist_control` node to command the Amiga robot with `TwistStamped` messages.
This bridge node will forward your `TwistStamped` messages published on the `/amiga/cmd_vel` topic
to the canbus service to drive the amiga.

To run the `twist_control` bridge:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch amiga_ros_bridge twist_control.launch
```

You can also subscribe to measured `TwistStamped` states of the amiga with ROS command line tools.

```bash
source ~/catkin_ws/devel/setup.bash
rostopic echo /amiga/vel
```

#### Use the `twist_wasd.py` example

To test the `twist_control` node of the `amiga-ros-bridge`,
you can publish `TwistStamped` commands to the ROS bridge on the
`/amiga/cmd_vel` topic with the `examples/twist_wasd.py` example.

> To successfully run this example, you must use your local PC,
> as the example won't work if executed directly from an Amiga brain
> (because of the opencv popup window).

Run from your terminal:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun amiga_ros_bridge examples/twist_wasd.py
```

Drive the robot with `WASD` keys to increment / decrement linear and angular velocities.
Hit the space bar to stop the robot (set linear and angular velocities to `0`).
