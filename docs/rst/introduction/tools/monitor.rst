.. _vulcanexus_monitor:

ROS 2 Monitor
=============

*ROS 2 Monitor* is a graphical desktop application shipped with *Vulcanexus Tools* aimed at monitoring DDS environments
deployed using the *eProsima Fast DDS* library, as it is the case in *ROS 2* ecosystems.
Thus, the user can monitor in real time the status of publication/subscription communications between *ROS 2* nodes.
They can also choose from a wide variety of communication parameters to be measured (latency, throughput, packet loss,
etc.), as well as record and compute in real time statistical measurements on these parameters (mean, variance,
standard deviation, etc.).

Furthermore, the user can check the status of the deployed *ROS 2* network at any time, i.e. see for each DDS Domain
which DomainParticipants are instantiated, as well as their publishers and subscribers and the topics under which they
publish or to which they subscribe respectively.
It is also possible to see the physical architecture of the network on which the *ROS 2* applications are running.

*ROS 2 Monitor* is designed to meet the following criteria:

#. **Monitoring**: real-time tracking of network status and communication.
#. **Intuitive**: graphical user interface developed following a user experience design approach.
#. **Introspection**: easily navigate through the deployed and active *ROS 2* nodes being able to inspect their
   configuration and physical deployment.
#. **Troubleshooting**: detect at a glance the possible issues or anomalous events that may occur in the communication.

.. warning::

    If having explicitly enabled statistics and statistical data are still not correctly received, only few data arrive
    or even none, configure the *ROS 2* nodes publishing statistics data with a less restrictive memory constraints.
    Please check the following
    `documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/statistics/dds_layer/troubleshooting.html#troubleshooting>`_
    for more details on how to do this.

.. raw:: html

    <video width=100% height=auto autoplay loop controls>
        <source src="_static/monitor_vulcanexus.mp4">
        Your browser does not support the video tag.
    </video>

Visit `eProsima Fast DDS Monitor Documentation <https://fast-dds-monitor.readthedocs.io/en/latest/>`_ for more
information on how to use this application. A hands-on example is also available in :ref:`this <tutorials_tools_fastdds_monitor>`
link, which provides step-by-step instructions on how to use *Vulcanexus Tools* for monitoring a *ROS 2* talker/listener
demo.
