FROM eprosima/vulcanexus:jazzy

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-jazzy-demo-nodes-cpp net-tools iptables

COPY listener_configuration.xml server_configuration.xml talker_configuration.xml ./
