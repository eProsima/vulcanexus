.. _dds2vulcanexus_topic:

Fast DDS - Vulcanexus Topic Intercommunication
==============================================

This tutorial presents a step-by-step demonstration on how to intercommunicate Vulcanexus applications with native Fast DDS applications.

.. contents::
    :depth: 2
    :local:
    :backlinks: none

.. _dds2vulcanexus_topic_background:

Background
----------

Being Fast DDS the default Vulcanexus middleware enables the possibility of intercommunicating Vulcanexus applications with native Fast DDS ones.
This is of special interest when integrating pre-existing systems with each other, such as interfacing with a third-party software which exposes a DDS API.
Since both Fast DDS' and Vulcanexus' backbone is DDS, it is possible to intercommunicate full-blown systems running a Vulcanexus stack with smaller systems for which Vulcanexus is either unnecessary or unfit, such as more constrained environments or applications that would not require any Vulcanexus functionality other than the middleware.

.. uml::
    :align: center

    package "Controller MCU" {
        [Fast DDS Motor controller] as controller
    }

    package "Robot's main MCU" {
        [Vulcanexus path planner] => controller : <<DDS>>
    }

.. _dds2vulcanexus_topic_prerequisites:

Prerequisites
-------------

For convenience, this tutorial is built and run within a Docker environment, although Docker is not required.
The tutorial focuses on the explanations regarding message type and topic name compatibilities rather than given an in depth explanation about the code used.
Create a clean workspace and download the *Vulcanexus - Fast DDS Topic Intercommunication* project:

.. code-block:: bash

    # Create directory structure
    mkdir ~/vulcanexus_dds_ws
    cd ~/vulcanexus_dds_ws
    mkdir fastdds_app idl vulcanexus_app
    # Download project source code
    wget -O CMakeLists.txt https://raw.githubusercontent.com/eProsima/vulcanexus/docs/dds2vulcanexus/docs/resources/tutorials/core/deployment/dds2vulcanexus/topic/CMakeLists.txt
    wget -O Dockerfile https://raw.githubusercontent.com/eProsima/vulcanexus/docs/dds2vulcanexus/docs/resources/tutorials/core/deployment/dds2vulcanexus/topic/Dockerfile
    wget -O package.xml https://raw.githubusercontent.com/eProsima/vulcanexus/docs/dds2vulcanexus/docs/resources/tutorials/core/deployment/dds2vulcanexus/topic/package.xml
    wget -O README.md https://raw.githubusercontent.com/eProsima/vulcanexus/docs/dds2vulcanexus/docs/resources/tutorials/core/deployment/dds2vulcanexus/topic/README.md
    wget -O fastdds_app/subscriber.cpp https://raw.githubusercontent.com/eProsima/vulcanexus/docs/dds2vulcanexus/docs/resources/tutorials/core/deployment/dds2vulcanexus/topic/fastdds_app/subscriber.cpp
    wget -O idl/HelloWorld.idl https://raw.githubusercontent.com/eProsima/vulcanexus/docs/dds2vulcanexus/docs/resources/tutorials/core/deployment/dds2vulcanexus/topic/idl/HelloWorld.idl
    wget -O vulcanexus_app/publisher.cpp https://raw.githubusercontent.com/eProsima/vulcanexus/docs/dds2vulcanexus/docs/resources/tutorials/core/deployment/dds2vulcanexus/topic/vulcanexus_app/publisher.cpp

The resulting directory structure should be:

.. code-block::

    ~/vulcanexus_dds_ws/
    ├── CMakeLists.txt
    ├── Dockerfile
    ├── fastdds_app
    │   └── subscriber.cpp
    ├── idl
    │   └── HelloWorld.idl
    ├── package.xml
    ├── README.md
    └── vulcanexus_app
        └── publisher.cpp

Finally, the Docker image can be built with:

.. code-block:: bash

    cd ~/vulcanexus_dds_ws
    docker build -f Dockerfile -t dds2vulcanexus .

.. _dds2vulcanexus_topic_vulcanexus:

Vulcanexus Application
----------------------

.. _dds2vulcanexus_topic_fastdds:

Fast DDS Application
--------------------

.. _dds2vulcanexus_topic_run:

Run the demo
------------

