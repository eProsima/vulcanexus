.. include:: ../../../../exports/alias.include

.. _topic_keys_tutorial:

Topic Keys Tutorial
===================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

In Vulcanexus, a *Topic* is a communication channel used for publishing and subscribing to updates of object states.
The value of data associated with a topic changes over time and each of these values are known as *data samples*.
:ref:`topic_keys` refer to topics where each data sample can update specific parts of the entire object state described by the topic (known as *instance*).

Unlike standard topics, where each data sample updates the entire object state with every data sample, keyed topics allow the user to reduce the number of required resources (topics, along with its associated publisher and subscriber) by multiplexing into a single one.
Please, refer to the documented section on :ref:`topic_keys` for a more detailed explanation.

Creating custom IDL messages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In ROS 2, users can define their own :ref:`custom messages<CustomInterfaces>`.
In addition, it is also possible to create custom messages using the `IDL (Interface Definition Language) <https://www.omg.org/spec/IDL/4.2/About-IDL>`_ format following a standardized way of defining the message structure.
Next example depicts how to define a custom message using the IDL format in ROS 2:

.. code-block:: bash

    # ExampleMsg.idl
    module package_name {
      module msg {
        struct ExampleMsg {
          string field;
          short another_field;
          double yet_another_field;
        };
      };
    };

    # ExampleMsg.msg equivalent
    string field;
    short another_field;
    double yet_another_field;

As it can be seen, using the IDL format requires naming the upper module with the same name as the package name containing it.
Furthermore, the message structure shall be named as the file containing it, and it shall be declared within an *msg* module nested in the package name one.

One of the advantages of defining messages in Interface Definition Language (IDL) is the ability to use annotations.
Annotations are metadata to the data structure definition that provide additional information about IDL constructs such as modules, interfaces, operations, attributes, and data types.
They are relevant for code generation, documentation, or other purposes.
Annotations in IDL typically follow the ``@`` symbol and can be applied to various IDL constructs.
Please, refer to the the tutorial about :ref:`communicating Vulcanexus and Fast DDS <dds2vulcanexus_topic>`, where the way to generate types from IDL files is shown.

The ``@key`` annotation is used to designate a member as key, which is covered in the following section.

Creating Keyed Messages
^^^^^^^^^^^^^^^^^^^^^^^

Keyed topics exist when one or more fields in the data structure are annotated as keys.
These key fields serve as unique identifiers for topic instances.
to organize and manage the data samples, facilitating efficient access, retrieval, and filtering
of data based on the specified key criteria.

The ``@key`` annotation designates a field as a key for the given topic type, which can have zero or more key fields
and can be applied to structure fields of various types:

* Any primitive, such as booleans, integers, characters, strings or sequences.
* Other existing or defined messages. If those have defined an inner key member, then those fields will we used as part of the key.
  Otherwise, the key will be the concatenation of all the fields.

In order to specify multiple keys, separate ``@key`` annotations are used.
The following example shows how to define a keyed message using the IDL format in Vulcanexus:

.. code-block:: bash

    # KeyedMsgName.idl
    module package_name {
      module msg {
        struct KeyedMsgName {
          @key long key;
          string data;
        };
      };
    };

.. note::

    Currently, the only supported message format that can be annotated with ``@key`` is *.idl*.
    Neither ``.msg`` nor ``.srv`` files support annotations yet.

Prerequisites
-------------

* It is recommended to have a basic understanding of ``Topic Keys`` introduced in the :ref:`topic_keys` section.
* An up-to-date Vulcanexus installation using one of the following installation methods:

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
    it is necessary to have installed the ``vucanexus-iron-base`` package,
    since this is the one that includes all the simulation tools, demos and tutorials.

    Source the following file to setup the Vulcanexus environment:

    .. code-block:: bash

            source /opt/vulcanexus/{DISTRO}/setup.bash


Retrieving the sources
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    # Create directory structure
    mkdir -p ~/vulcanexus_ws/src/demo_keys_cpp
    mkdir ~/vulcanexus_ws/src/demo_keys_cpp/src
    mkdir ~/vulcanexus_ws/src/demo_keys_cpp/msg
    cd ~/vulcanexus_ws/src/demo_keys_cpp

    # Download demo package source code
    wget -O CMakeLists.txt https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/CMakeLists.txt
    wget -O package.xml https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/package.xml
    wget -O README.md https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/README.md
    wget -O msg/SensorDataMsg.idl https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/msg/SensorDataMsg.idl
    wget -O src/right_sensor.cpp https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/src/right_sensor.cpp
    wget -O src/left_sensor.cpp https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/src/left_sensor.cpp
    wget -O src/controller.cpp https://raw.githubusercontent.com/eProsima/vulcanexus/{DISTRO}/docs/resources/tutorials/core/deployment/keys/src/controller.cpp

The resulting directory structure should be:

.. code-block::

    ~/vulcanexus_ws
     ├──src
        ├── demo_keys_cpp
            ├── CMakeLists.txt
            ├── README.md
            ├── msg
            │   └── SensorDataMsg.idl
            ├── package.xml
            └── src
                ├── left_sensor.cpp
                ├── right_sensor.cpp
                └── controller.cpp

Building the demo package
^^^^^^^^^^^^^^^^^^^^^^^^^

Once the environment has been setup and the demo package sources are available, we are ready to build the workspace.
Get into the root of the workspace and build it with the following commands:

.. code-block:: bash

    source /opt/vulcanexus/{DISTRO}/setup.bash
    cd ~/vulcanexus_ws
    colcon build


Running the demo
----------------

Finally, we can run the demo by executing the following commands in separate terminals:

.. tabs::

    .. tab:: Shell 1 (Right Sensor)

        .. code-block:: bash

            source ~/vulcanexus_ws/install/setup.bash
            ros2 run demo_keys_cpp right_sensor

    .. tab:: Shell 2 (Left Sensor)

        .. code-block:: bash

            source ~/vulcanexus_ws/install/setup.bash
            ros2 run demo_keys_cpp left_sensor

    .. tab:: Shell 3 (Controller)

        .. code-block:: bash

            source ~/vulcanexus_ws/install/setup.bash
            ros2 run demo_keys_cpp controller

.. note::

    If a docker deployment was preferred, it would be necessary to attach the other two terminals to
    the running docker container before executing the above commands.
    This can be done by running ``docker exec -it <container_name> /bin/bash``.

The resulting output should be similar to the following:

.. raw:: html

    <video width=100% height=auto autoplay loop controls muted>
        <source src="../../../../../_static/resources/tutorials/core/deployment/keys/topic_keys_tutorial.mp4">
        Your browser does not support the video tag.
    </video>

One of the important differences to note against having used a standard topic is that, setting a reliable
communication and defining history QoS (Quality of Service) as ``KEEP_LAST 1``, if the controller late joins the application, it is
by no means guaranteed to receive the last data published by both of the sensors whereas by using topic keys
we assure that the last status of each instance (sensor) is received. This is because QoS are applied per data instance.
This and further benefits can be explored in :ref:`benefits_of_topic_keys`.