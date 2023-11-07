#!/bin/bash

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Docker is not installed. Please install Docker and try again."
    exit 1
fi

# Define the container name
container_name="micro-ros-agent"

# Check if the container is already running
if sudo docker ps -q --filter "name=$container_name" | grep -q .; then
    echo "The '$container_name' container is already running."
else
    # Start the micro-ROS Agent container as administrator using 'sudo'
    sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -v6
fi