.. _vulcanexus_shapes_demo:

ROS 2 Shapes Demo
=================

eProsima Shapes Demo is an application in which Publishers and Subscribers are shapes of different colors and sizes moving on a board. Each shape refers to its own topic: Square, Triangle or Circle.
A single instance of the eProsima Shapes Demo can publish on or subscribe to several topics at a time.

It demonstrates the capabilities of eProsima Fast DDS or as a proof of interoperability with other RTPS-compliant implementations.

After the executable is launched, a window similar to the one presented in the following image should be displayed.

.. figure:: /rst/figures/intro/tools/shapes_demo/mainWindow.png


Publishing a Topic
------------------

The Publish button allow the users to define the Shape (topic) and Quality of Service (QoS) for their publication.
The following image shows an example of the Publication menu.

.. figure:: /rst/figures/intro/tools/shapes_demo/publish.png

There are multiple parameters that the user can define in this menu:

* Shape: This parameter defines the topic where the publication is going to occur. Three different shapes can be published: Square, Circle and Triangle (see Fast DDS Topic Documentation).
* Color: The user can define the color of the shape. This parameter will be used as key; that is, a way to distinguish between multiple instances of the same shape (see Fast DDS Topics with key documentation).
* Size: This parameter allows to control the size of the shape. The size can vary between 1 and 99.
* Partition: The user can select different partitions to differentiate groups of publishers and subscribers. The user can select between four partitions (A, B, C and D). Additionally the user can select the * partition, that will be matched against all other partitions (see Fast DDS Partitions Documentation).

.. note::
    Using the wildcard (*) partition is not the same as not using any partition. A publisher that uses the wildcard partition will not be matched with a subscriber that do not defines any partitions.

* Reliable: The user can select to disable the Reliable check-box to use a Best-Effort publisher (see Fast DDS ReliabilityQosPolicy Documentation).
Subscribing to a Topic
----------------------

.. figure:: /rst/figures/intro/tools/shapes_demo/subscribe.png

Next Steps
----------

Visit `eProsima Shapes Demo Documentation <https://eprosima-shapes-demo.readthedocs.io/en/latest/>`_ for more information on how to use this application.
