FROM eprosima/vulcanexus:iron

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-iron-demo-nodes-cpp net-tools iptables

COPY listener_configuration.xml server_configuration.xml talker_configuration.xml ./
