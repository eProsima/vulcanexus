FROM eprosima/vulcanexus:humble

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-demo-nodes-cpp net-tools iptables

COPY listener_configuration.xml server_configuration.xml talker_configuration.xml ./
