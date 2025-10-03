# Running ROS GUI tools remotely using X11 forwarding

This tutorial will guide you through the process of running ROS GUI tools remotely using X11 forwarding. This is useful when you want to run a GUI tool on a remote machine, but you want to see the GUI on your local machine.

## Prerequisites

1. You need to have X11 server running on your local machine. If you are on Linux, you already have it. If you are on macOS, you can use [XQuartz](https://www.xquartz.org/).
2. You need to have SSH access to the remote machine.

### Additional tools

On local Linux machine

```bash
sudo apt-get update && sudo apt-get install -y x11-apps xauth
```

## Configure SSH to forward X11

```bash
Host dr-qp-24
  # ... other settings
  ForwardX11 yes
```

or

```bash
ssh -X user@raspi-ip
```

## Testing X11 forwarding

Install x11-apps on remote machine

```bash
sudo apt-get update && sudo apt-get install -y x11-apps
```

Test X11 forwarding by running

```bash
xeyes
```

You should see the eyes following your cursor.

## Running rqt on remote machine

If ROS is running in docker, you need to run the following command to forward X11 to the docker container:

```bash
docker run -it \
    -e DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    --env="FASTDDS_BUILTIN_TRANSPORTS=UDPv4" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --network host --ipc host --pid host \
    ghcr.io/dr-qp/jazzy-ros-desktop:edge \
    rqt
```

If ROS is installed locally, you can run `rqt` directly.
