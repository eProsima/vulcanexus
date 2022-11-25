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

.. _dds2vulcanexus_topic_vulcanexus:

Vulcanexus Application
----------------------

.. _dds2vulcanexus_topic_fastdds:

Fast DDS Application
--------------------

.. _dds2vulcanexus_topic_run:

Run the demo
------------

