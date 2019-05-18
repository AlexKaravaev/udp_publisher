# ROS package for reading udp data from socket

## Description
This package enables reading from udp socket, constructing message and publishing it to user-specified topic.
## Installation

1. Put contents of this repo into `[your_catkin_ws_path]/src/` and run `catkin_make`

## Usage

1. Launching only this node. For example with device_ip=127.0.0.1, port_number = 54000, topic_name=socket_data, queue_size=1
```shell
roslaunch udp_publisher udp_publisher.launch device_ip:=127.0.0.1 port_number:=54000 queue_size:=1 udp_topic:=socket_data
```
2. Including this roslaunch file in another package. Just past below into some existing bigger roslaunch file.
```xml

  <include file="$(find udp_publisher)/launch/udp_publisher.launch">
     <arg name="device_ip" value="127.0.0.1" />
     <arg name="port_number" value="54000"/>
     <arg name="udp_topic" value="socket_data"/>
     <arg name="queue_size" value="1"/>
  </include>
```