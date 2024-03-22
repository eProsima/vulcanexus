.. include:: ../../../../exports/alias.include

.. _filtered_topic_keys_tutorial:

Topic Instances Subscription Filtering Tutorial
===============================================

This tutorial aims to demonstrate how to receive data only from certain topic instances by combining the use of topic keys and topic content filtering.

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

In Vulcanexus, topics are a mean for representing the state of an object.
:ref:`topic_keys` refer to topics where each data sample represent an update of the state of a specific object  (known as *instance*) among all those objects represented in the topic.

Keyed topics allow the user to reduce the number of required resources (topics, along with its associated publisher and subscriber) by multiplexing updates of several objects of the same kind into a single resource.
Please, refer to the documented section on :ref:`topic_keys` for a more detailed explanation.

The `Content Filter Topic <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/contentFilteredTopic/contentFilteredTopic.html>`_ facilitates efficient data distribution by allowing the subscription (reader-side) to specify criteria for the types of data they wish to receive.
By defining this criteria, irrelevant data can be filtered out and applications can focus only on the information that is relevant to their needs.
This functionality not only reduces the amount of data transmitted over the network but also minimizes processing overhead on the receiving end, leading to improved system performance and scalability.

When combined with topic instances, the benefits of the Content Filter are further enhanced.
By associating specific filter criteria with each topic instance, it is possible fine-tune the data selection process and tailor it to their precise requirements.
This granular level of filtering enables applications to optimally manage data while ensuring that they exchange only the information that is pertinent to their individual use cases.

.. image:: ../../../../figures/enhancements/keys/keyed-topics-cft.gif

Prerequisites
-------------

* It is highly recommended to have completed the :ref:`topic_keys_tutorial`.
* An up-to-date (latest) Vulcanexus installation using one of the following installation methods:

  * :ref:`linux_binary_installation`
  * :ref:`linux_source_installation`
  * :ref:`docker_installation`

Preparing the demo package
--------------------------

Lets start by setting up the Vulcanexus environment.
For this, there are two possible options:

#.  Running the Vulcanexus Docker image.

    Run the Vulcanexus Docker image with:

    .. code-block:: bash

        docker run -it --rm ubuntu-vulcanexus:{DISTRO}-desktop

    Then, within the container, source the Vulcanexus installation with:

    .. code-block:: bash

            source /opt/vulcanexus/{DISTRO}/setup.bash

#.  Running the tutorial on the local host. For this second option,
    it is necessary to have the ``vucanexus-iron-base`` package installed.

    Source the following file to setup the Vulcanexus environment:

    .. code-block:: bash

            source /opt/vulcanexus/{DISTRO}/setup.bash


Retrieving the sources
^^^^^^^^^^^^^^^^^^^^^^

Create a new workspace and download the demo package sources as indicated below:

.. code-block:: bash

    # Create directory structure
    mkdir -p ~/vulcanexus_ws/src/demo_keys_filtering_cpp
    mkdir ~/vulcanexus_ws/src/demo_keys_filtering_cpp/msg
    mkdir ~/vulcanexus_ws/src/demo_keys_filtering_cpp/src
    mkdir ~/vulcanexus_ws/src/demo_keys_filtering_cpp/launch
    cd ~/vulcanexus_ws/src/demo_keys_filtering_cpp

    # Download demo package source code
    wget -O CMakeLists.txt https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/demo_keys_filtering_cpp/CMakeLists.txt
    wget -O package.xml https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/demo_keys_filtering_cpp/package.xml
    wget -O README.md https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/demo_keys_filtering_cpp/README.md
    wget -O msg/KeyedSensorDataMsg.idl https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/demo_keys_filtering_cpp/msg/KeyedSensorDataMsg.idl
    wget -O src/filtered_keyed_sensor.cpp https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/demo_keys_filtering_cpp/src/filtered_keyed_sensor.cpp
    wget -O src/filtered_keyed_controller.cpp https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/demo_keys_filtering_cpp/src/filtered_keyed_controller.cpp
    wget -O launch/keyed_sensors_launch.py https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/demo_keys_filtering_cpp/launch/keyed_sensors_launch.py

The resulting directory structure should be:

.. code-block::

    ~/vulcanexus_ws
     ├──src
        ├── demo_keys_filtering_cpp
            ├── CMakeLists.txt
            ├── README.md
            ├── launch
            │   └── keyed_sensors_launch.py
            ├── msg
            │   └── KeyedSensorDataMsg.idl
            ├── package.xml
            └── src
                ├── filtered_keyed_controller.cpp
                └── filtered_keyed_sensor.cpp

A brief analysis on the provided files is explained below:

* *demo_keys_filtering_cpp* : This directory contains the main source code and configuration files for the demonstration.
* ``CMakeLists.txt``: This file is used with CMake to specify build configurations and dependencies.
* ``README.md``: This is a markdown file providing instructions or information about the demonstration.
* *launch*: This directory contains launch configuration files for launching ROS nodes.
  * ``keyed_sensors_launch.py``: This Python script is used to launch the demonstration nodes.
* *msg*: This directory contains message definition files.
  * KeyedSensorDataMsg.idl: This is an IDL (Interface Definition Language) file defining the message structure for keyed sensor data.
* ``package.xml``: This is an XML file containing metadata about the ROS package.
* *src*: This directory contains the source code files for the demonstration.
  * ``filtered_keyed_controller.cpp``: This is the source code for a controller node that filters keyed sensor data in reception, being the most relevant lines the ones that define the filter expression and Quality of Service settings:

.. code-block:: bash

    // Initialize a subscription with a content filter to receive data from sensors 2 to 4
    rclcpp::SubscriptionOptions sub_options;
    sub_options.content_filter_options.filter_expression =
        "sensor_id >= 2 AND sensor_id <= 4 AND measurement > %0";
    sub_options.content_filter_options.expression_parameters = {
      std::to_string(SENSOR_TRIGGER)
    };

    // Create the subscription with the content filter options
    sub_ = create_subscription<demo_keys_filtering_cpp::msg::KeyedSensorDataMsg>("/robot/sensors",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        callback,
        sub_options);

* ``filtered_keyed_sensor.cpp``: This is the source code for a sensor node that publishes keyed sensor data.
  The most relevant lines are the ones that create the publication with a particular Quality of Service settings that enables the controller to late join the application but still receiving the latest update for every instance with the use of topic keys.

.. code-block:: bash

  pub_ = this->create_publisher<demo_keys_filtering_cpp::msg::KeyedSensorDataMsg>(
        "/robot/sensors",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

Building the demo package
^^^^^^^^^^^^^^^^^^^^^^^^^

Once the environment has been setup and the demo package sources are available, the demo package can be built.
Get into the root of the workspace and build it with the following commands:

.. code-block:: bash

    source /opt/vulcanexus/{DISTRO}/setup.bash
    cd ~/vulcanexus_ws
    colcon build


Running the demo
----------------

In the demo, different sensors are publishing data
Run the demo by executing the following commands in separate terminals:

.. note::

    If a docker deployment was preferred, it would be necessary to attach the other two terminals to
    the running docker container before executing the above commands.
    This can be done by running ``docker exec -it <container_name> /bin/bash``.

.. tabs::

    .. tab:: Shell 1 (Sensors)

        .. code-block:: bash

            source ~/vulcanexus_ws/install/setup.bash
            ros2 launch demo_keys_filtering_cpp keyed_sensors_launch.py

    .. tab:: Shell 2 (Controller)

        .. code-block:: bash

            source ~/vulcanexus_ws/install/setup.bash
            ros2 run demo_keys_filtering_cpp filtered_keyed_controller


The resulting output should be similar to the following, in which the controller node is only receiving data from the specified sensors, i.e. sensors which sensor_id is in the range [2, 4].
In addition, only when the measurement is greater than 60, the controller node will receive data.
That is specified in the filter expression ``sensor_id >= 2 AND sensor_id <= 4``:

.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../../../_static/resources/tutorials/core/deployment/keys/filtered_keyed_topic.mp4">
        Your browser does not support the video tag.
    </video>

Even in a late-joining scenario, the controller node will receive the latest data of the sensors that meet the filtering criteria at the moment it joins the application, which could be crucial depending on the type of the real application.

Overall, the combination of topic instances with a content filter topic offers significant benefits in terms of data efficiency, scalability, adaptability and resource optimization.
By leveraging these capabilities, *Vulcanexus* applications can efficiently manage and distribute data in complex distributed environments.
These and further benefits can be explored in :ref:`benefits_of_topic_keys`.
Also, for additional materials regarding the topic content filter, please refer to the :doc:`Content Filtering Subscription Tutorial <../../../../../ros2_documentation/source/Tutorials/Demos/Content-Filtering-Subscription>`.

