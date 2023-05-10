FROM ros:humble

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-demo-nodes-cpp net-tools iptables

COPY node_configuration.xml .

COPY server_configuration.xml .
