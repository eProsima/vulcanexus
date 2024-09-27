FROM ubuntu:22.04
LABEL author=juanlopez@eprosima.com

# Avoid interactuation with installation of some package that needs the locale.
ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Avoids using interactions during building
ENV DEBIAN_FRONTEND=noninteractive

# Use a bash shell so it is possigle to run things like `source` (required for colcon builds)
SHELL ["/bin/bash", "-c"]

COPY linux_bisource_installation.bash /tmp/linux_bisource_installation.bash
RUN chmod +x /tmp/linux_bisource_installation.bash
RUN /tmp/linux_bisource_installation.bash
RUN rm /tmp/linux_bisource_installation.bash

COPY run.bash /root/run.bash
RUN chmod +x /root/run.bash

ENTRYPOINT ["/root/run.bash" ]
