# Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
FROM ubuntu-vulcanexus:kilted-desktop as build

ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

SHELL ["/bin/bash", "-c"]

WORKDIR /dds2vulcanexus_ws/src/dds2vulcanexus
COPY CMakeLists.txt package.xml README.md ./
COPY idl idl
COPY vulcanexus_app vulcanexus_app
COPY fastdds_app fastdds_app

WORKDIR /dds2vulcanexus_ws
RUN source /opt/vulcanexus/kilted/setup.bash && \
    colcon build --event-handlers=console_direct+

FROM ubuntu-vulcanexus:kilted-desktop as run-time

COPY --from=build /dds2vulcanexus_ws/install /dds2vulcanexus

RUN echo 'source /dds2vulcanexus/setup.bash' >> ~/.bashrc
RUN echo "alias vulcanexus_publisher=/dds2vulcanexus/dds2vulcanexus/lib/dds2vulcanexus/vulcanexus_publisher" >> ~/.bashrc
RUN echo "alias fastdds_subscriber=/dds2vulcanexus/dds2vulcanexus/lib/dds2vulcanexus/fastdds_subscriber" >> ~/.bashrc
