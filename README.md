# amiga-ros-bridge

## Setup

These instructions are for installing the amiga-ros-bridge
on Amiga brains running the Amiga OS 2.0.

For Amiga brains running Amiga OS `1.0` - `1.3`,
please refer to
[github.com/farm-ng/amiga-ros-bridge-v1](https://github.com/farm-ng/amiga-ros-bridge-v1).

### `ssh` into your robot

This requires having set up your user credentials.

> See [Access and Develop on the Brain](https://amiga.farm-ng.com/docs/ssh/)
for details / assistance gaining `ssh` user access.

```bash
ssh <your-robot>
```

The remaining setup steps assume you are `ssh`'d into the robot.

### Pull the container

```bash
# Pull the container
# This may take 20 minutes, depending on network connection.
docker pull dustynv/ros:noetic-pytorch-l4t-r35.2.1

# Check that the image exists
docker images

# You should see similar to:
# REPOSITORY            TAG                          IMAGE ID       CREATED         SIZE
# dustynv/ros           noetic-pytorch-l4t-r35.2.1   1ccdc74cd9c6   6 months ago    13.6GB
```

### Make a directory to bind mount

This will create a directory that you will bind mount when you run your ROS docker container.
That means you will have shared access to the files as your `ssh` user
and when in the ROS docker container.
By creating this directory ahead of time, you will have more file permissions as your user than if you create the directory inside
the ROS docker container.

```bash
# You can choose a different name for `catkin_ws` here
# But the remaining documentation assumes this is what you've used
mkdir -p ~/catkin_ws/src
```

### Clone the amiga-ros-bridge

```bash
cd ~/catkin_ws/src/
git clone --recursive https://github.com/farm-ng/amiga-ros-bridge.git
```

### Build the venv

```bash
./amiga-ros-bridge/setup_venv.sh
```

### Start the container

See **Details on `docker run` command** below for more information.

```bash
docker run --runtime nvidia -it --rm --network=host -v ~/catkin_ws:/workspace/catkin_ws dustynv/ros:noetic-pytorch-l4t-r35.2.1
```

### Setup your catkin workspace

> These are based on related the [ROS wiki docs](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

```bash
# Navigate to the bind mount catkin_ws
cd /workspace/catkin_ws/

# Run catkin_make
catkin_make
```

### Add `venv` to `devel/setup.bash`

```bash
# Navigate to the bind mount catkin_ws
cd /workspace/catkin_ws/

# Get the current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Add sourcing the amiga-ros-bridge to devel/setup.bash
echo "source $DIR/src/amiga-ros-bridge/source_venv.sh" >> devel/setup.bash
```

So that when you `source devel/setup.bash`,
the `amiga-ros-bridge/venv/` will also be sourced.

> NOTE:
>
> `apt` or `pip` packages installed while inside the dustynv docker container will not persist when you exit / re-enter.
> The `venv` allows you to bypass this constraint with `pip` packages.

## Run the amiga-ros-bridge

```bash
source /workspace/catkin_ws/devel/setup.bash
```

### TODO: Add it and run it

TODO!

## Extra details / context

### Details on `docker run` command

- `docker run`: This is the basic command to run a Docker container.

- `--runtime nvidia`: This flag specifies the use of the NVIDIA GPU runtime. It's typically used when you need GPU support within the container.

- `-it`: These are combined flags that make the container run in interactive mode (`-i`) and allocate a pseudo-TTY (`-t`) for terminal interaction.

- `--rm`: This flag indicates that the container should be removed (cleaned up) when it exits. Useful for temporary containers or for situations where you don't need to keep the container around after use.

- `--network=host`: This flag instructs the container to share the host network namespace, allowing the container to access the network as if it were the host. It's often used when the container needs to interact with the host network.

- `-v ~/catkin_ws:/workspace/catkin_ws`: This is a volume or bind mount configuration. It maps the `~/catkin_ws` directory on your host machine to the `/workspace/catkin_ws` directory inside the container. Any changes made in the container's `/workspace/catkin_ws` directory will be reflected in the `~/catkin_ws` directory on your host machine.

- `dustynv/ros:noetic-pytorch-l4t-r35.2.1`: This is the name of the Docker image to be used when creating the container. It specifies the image's repository (`dustynv/ros`) and its tag (`noetic-pytorch-l4t-r35.2.1`). Docker images are like snapshots of the container's filesystem.

### Details on container selection

### TODO: REMOVE THIS SECTION

> NOTE: The reason I went for the pytorch container is that it seems to have all the required ros packages already installed.
> Other versions require `apt update && apt install ros-noetic-ros-base` for using some basic tools, like `rostopic`
> But these installed apt packages do not persist between runs of the container!
>
> Feel free to check the other `noetic` containers at https://github.com/dusty-nv/jetson-containers/tree/master/packages/ros
>
> Keep in mind it is important to get the correct version
> If you want to check versions, as your user account:
>
> ```bash
> cat /etc/nv_tegra_release
> # Should indicate R35 (release), REVISION: 2.1
>
> dpkg -l | grep nvidia-jetpack
> # Should indicate Jetpack 5.1
>
> lsb_release -a
> # Should indicate Ubuntu 20.04
> ```
