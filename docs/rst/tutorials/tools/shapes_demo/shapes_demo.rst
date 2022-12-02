.. _tutorials_tools_shapes_demo:

ROS 2 Shapes Demo
=================

.. contents::
    :depth: 2
    :local:
    :backlinks: none

Background
----------

.. note::
    This is under maintenance and will be updated soon.

Prerequisites
-------------

Ensure that the Vulcanexus installation includes Vulcanexus Tools (either ``vulcanexus-humble-desktop``, ``vulcanexus-humble-tools``, or ``vulcanexus-humble-base``).
Also, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/humble/setup.bash


Launch ROS 2 Shapes Demo
-----------------------

Initiate *ROS 2 Shapes Demo* running the following command:

.. code-block:: bash

    shapesdemo

Tutorials
---------

.. note::
    This is under maintenance and will be updated soon.

Discovery and basic connectivity
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In *Fast DDS*, the discovery task is automatic.
*Fast DDS* performs the task of finding the relevant information and distributing it to its destination.
It means that new nodes are automatically discovered by any other in the network.
Please refer to the
`Fast DDS Discovery Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html>`_
for more information on the various *Fast DDS* discovery mechanisms.

In this test, three Publishers and three Subscribers are launched.
At the end, two additional squares will be displayed in each window, reflecting the movements of the original square in
real time.
That is, subscribers subscribing to the "Square" topics are matched with the publishers of the other instances.

Step-by-step example implementation
"""""""""""""""""""""""""""""""""""

First, three publishers must be created.

1. Create a red square publisher:

   - Start eProsima Shapes Demo (this instance will be referred to as *Instance1*).
   - Click on Publish.
   - Select SQUARE option for Shape and RED for Color.

2. Create a blue square publisher:

   - Start eProsima Shapes Demo (this instance will be referred to as *Instance2*).
   - Click on Publish.
   - Select SQUARE option for Shape and BLUE for Color.

3. Create a black square publisher:

   - Start eProsima Shapes Demo (this instance will be referred to as *Instance3*).
   - Click on Publish.
   - Select SQUARE option for Shape and BLACK for Color.

The current setting should be similar to that shown in the figure below.

.. note::
    Add figure.

Then, three subscribers must be created.

1. Click Subscribe on *Instance1*.

   - Select SQUARE option for Shape.
   - Change the History field from 6 to 1.

2. Click Subscribe on *Instance2*.

   - Select SQUARE option for Shape.
   - Change the History field from 6 to 1.

3. Click Subscribe on *Instance3*.

   - Select SQUARE option for Shape.
   - Change the History field from 6 to 1.

The eProsima Shapes Demo windows should look similar to the following image.

.. note::
    Add figure.

History and Durability
^^^^^^^^^^^^^^^^^^^^^^

A publisher can send messages throughout a Topic even if there are no DataReaders on the network.
Moreover, a DataReader that joins to the Topic after some data has been written could be interested in accessing that
information.
The durability defines how the system will behave regarding those samples that existed on the Topic before the
subscriber joins.
Please refer to
`Fast DDS DurabilityQosPolicy Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#durabilityqospolicy>`_
for more information on Durability QoS.

In the following example, the publishers' history is set to ``KEEP_LAST``, and
there are two options for the durability configuration which are ``VOLATILE`` and ``TRANSIENT_LOCAL``.
If ``VOLATILE`` is selected, the previous data samples will not be sent.
However, if ``TRANSIENT_LOCAL`` is selected, the :math:`n^{th}` previous data samples will be sent to the late-joining
subscriber.

In this example, one hundred red squares will be displayed in *Instance2* and *Instance3*, reflecting the movements of
the red square of the publisher from *Instance1*.
The leading square indicates the current position of the published square.

Step-by-step example implementation
"""""""""""""""""""""""""""""""""""

First, three instances are launched and a publisher is created in each of them:

1 - Create a red square publisher:

   - Start eProsima Shapes Demo (this instance will be referred to as *Instance1*).
   - Click on Publish.
   - Select SQUARE option for Shape and RED for Color.
   - Change the History field from 6 to 100.
   - Select TRANSIENT_LOCAL.

2 - Create an orange square publisher:

   - Start eProsima Shapes Demo (this instance will be referred to as *Instance2*).
   - Click on Publish.
   - Select SQUARE option for Shape and ORANGE for Color.
   - Change the History field from 6 to 100.
   - Select TRANSIENT_LOCAL.

3 - Create a black square publisher:

   - Start eProsima Shapes Demo (this instance will be referred to as *Instance3*).
   - Click on Publish.
   - Select SQUARE option for Shape and BLACK for Color.
   - Change the History field from 6 to 100.
   - Select TRANSIENT_LOCAL.

The eProsima Shapes Demo environment should look similar to the following figure.

.. note::
    Add figure.

Then, subscriber in each instance is created.

4. Click Subscribe on *Instance1*.

   - Select SQUARE option for Shape.
   - Change the History field from 6 to 100.

5. Click Subscribe on *Instance2*.

   - Select SQUARE option for Shape.
   - Change the History field from 6 to 100.

6. Click Subscribe on *Instance3*.

   - Select SQUARE option for Shape.
   - Change the History field from 6 to 100.

The eProsima Shapes Demo environment should look similar to the following figure.

.. note::
    Add figure.

Liveliness
^^^^^^^^^^

Content Based Filter
^^^^^^^^^^^^^^^^^^^^

Next Steps
----------

Visit `ROS 2 Shapes Demo <https://docs.vulcanexus.org/en/latest/rst/introduction/tools/shapes_demo.html>`_ for more information on how to use this application.
