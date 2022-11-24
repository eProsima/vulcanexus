.. _vulcanexus_shapes_demo:

ROS 2 Shapes Demo
=================

eProsima Shapes Demo is an application in which Publishers and Subscribers are shapes of different colors and sizes moving on a board. Each shape refers to its own topic: Square, Triangle or Circle.
A single instance of the eProsima Shapes Demo can publish on or subscribe to several topics at a time.

It demonstrates the capabilities of eProsima Fast DDS or as a proof of interoperability with other RTPS-compliant implementations.

Using eProsima Shapes Demo
--------------------------

After the executable is launched, a window similar to the one presented in the following image should be displayed.

.. figure:: /rst/figures/intro/tools/shapes_demo/mainWindow.png


Publishing a Topic
^^^^^^^^^^^^^^^^^^

The Publish button allow the users to define the Shape (topic) and Quality of Service (QoS) for their publication.
The following image shows an example of the Publication menu.

.. figure:: /rst/figures/intro/tools/shapes_demo/publish.png

There are multiple parameters that the user can define in this menu:

* Shape: This parameter defines the topic where the publication is going to occur. Three different shapes can be published: ``Square``, ``Circle`` and ``Triangle`` (see Fast DDS Topic Documentation).
* Color: The user can define the color of the shape. This parameter will be used as key; that is, a way to distinguish between multiple instances of the same shape (see Fast DDS Topics with key documentation).
* Size: This parameter allows to control the size of the shape. The size can vary between ``1`` and ``99``.
* Partition: The user can select different partitions to differentiate groups of publishers and subscribers. The user can select between four partitions (``A``, ``B``, ``C`` and ``D``). Additionally the user can select the ``*`` partition, that will be matched against all other partitions (see Fast DDS Partitions Documentation).

.. note::
    Using the wildcard (``*``) partition is not the same as not using any partition. A publisher that uses the wildcard partition will not be matched with a subscriber that do not defines any partitions.

* Reliable: The user can select to disable the ``Reliable`` check-box to use a ``Best-Effort`` publisher (see Fast DDS ReliabilityQosPolicy Documentation).
* History and Durability: The publishers’s History is set to ``KEEP_LAST``. The user can select the number of samples that the publisher is going to save and whether this History is going to be ``VOLATILE`` or ``TRANSIENT_LOCAL``. The latter will send that last stored values to subscribers joining after the publisher has been created. (see Fast DDS DurabilityQosPolicy Documentation).
* Liveliness: The user can select the Liveliness QoS of the publisher from three different values: ``AUTOMATIC``, ``MANUAL_BY_PARTICIPANT`` and ``MANUAL_BY_TOPIC``. The Lease Duration value and Announcement Period can also be configured. The latter only applies if Liveliness is set to AUTOMATIC or MANUAL_BY_PARTICIPANT (see Fast DDS LivelinessQosPolicy Documentation).
* Ownership: The Ownership QoS determines whether the key (color) of a Topic (Shape) is owned by a single publisher. If the selected ownership is ``EXCLUSIVE`` the publisher will use the Ownership strength value as the strength of its publication. Only the publisher with the highest strength can publish in the same Topic with the same Key (see Fast DDS OwnershipQosPolicy Documentation).
* Deadline: The Deadline QoS determines the maximum expected amount of time between samples. When the deadline is missed the application will be notified and a message will be printed on the console (see Fast DDS DeadlineQosPolicy Documentation).
* Lifespan: The Lifespan QoS determines the duration while the sample is still valid. When a sample’s lifespan expires, it will be removed from publisher and subscriber histories. (see Fast DDS LifespanQosPolicy Documentation).

.. note::
    Using Lifespan QoS will not have any visual effect.

Subscribing to a Topic
^^^^^^^^^^^^^^^^^^^^^^

When the Subscriber button is pressed, a new window appear to allow the user to define the Shape (topic) and Quality of Service (QoS) for its subscription.
The following image shows an example of the Subscribe menu.

.. figure:: /rst/figures/intro/tools/shapes_demo/subscribe.png

This menu is highly similar to the Publication menu but the user cannot change the color and size of the Shape, and it has additional elements:

* Liveliness: This QoS policy is applied in the same way as in the publisher except for the Announcement Period, which does not apply for the Subscriber (see Fast DDS LivelinessQosPolicy Documentation).
* Time Based Filter: This value can be used by the user to specify the minimum amount of time (in milliseconds) that the subscriber wants between updates. (see Fast DDS TimeBasedFilterQosPolicy Documentation).
* Content Based Filter: This filter draws a rectangle in the instances window. Only the shapes that are included in this rectangle are accepted while the rest of them are ignored. The user can dynamically resize and move this content filter.

.. note::
    Using Lifespan QoS will not have any visual effect.

Participant configuration
^^^^^^^^^^^^^^^^^^^^^^^^^
The eProsima Shapes Demo application allows the user to define Participant policies.
To see the Options window, please go to Options->Participant Configuration in the main bar.
The following image shows the Options Menu.

.. figure:: /rst/figures/intro/tools/shapes_demo/participant.png

* Transport Protocol: You can select between UDP protocol, TCP LAN Server, TCP WAN Server or TCP Client, and Shared Memory protocol.

.. note::
    This is under maintenance and will be updated soon.

Preferences
^^^^^^^^^^^
The eProsima Shapes Demo application allows the user to define additional options.
To see the Options window, please go to Options->Preferences in the main bar.
The following image shows the Options Menu.

.. figure:: /rst/figures/intro/tools/shapes_demo/preferences.png

The user can customize several aspects of Shapes Demo operation:

* Update interval: This value changes the publication period for all the publishers.
* Speed: This scroll bar allows the user to change how much the Shape moves between two write calls.

Endpoints and Output tabs
^^^^^^^^^^^
.. note::
    This is under maintenance and will be updated soon.

Next Steps
^^^^^^^^^^

Visit `eProsima Shapes Demo Documentation <https://eprosima-shapes-demo.readthedocs.io/en/latest/>`_ for more information on how to use this application.
