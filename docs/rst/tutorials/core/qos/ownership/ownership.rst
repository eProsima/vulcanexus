.. _tutorials_qos_ownership_ownership:

Modifying Ownership and Ownership Strenght QoS Policy
=====================================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

Fast DDS over Vulcanexus offers the possibility of fully configuring QoS policy through XML profile definition.
This tutorial provides step-by-step instructions to modify the Ownership QoS within the ROS 2 talker/listener demo.

Prerequisites
-------------

Please, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash

In a terminal sourced with the previous line, run the following command to install the ROS 2 demo-nodes-cpp tutorial
(administrative privileges may be required):

.. code-block:: bash

    apt-get install -y ros-humble-demo-nodes-cpp

XML Profile definition
----------------------


Execute ROS 2 demo nodes with modified QoS
------------------------------------------


