.. _vulcanexus_binary_distribution:

Vulcanexus binary distributions
===============================

*Vulcanexus* provides several binary distributions that install specific packages depending on the developer's needs.
:ref:`vulcanexus_introduction` table summarizes the different *Vulcanexus* package distributions along with the most important included packages.

Vulcanexus Core
---------------

**Vulcanexus Core** is the most fundamental and minimal *Vulcanexus* installation.
It is composed by the `ROS 2 Base libraries <https://ros.org/reps/rep-2001.html#end-user-entry-points>`_ but with the latest `eProsima Fast DDS <https://fast-dds.docs.eprosima.com/en/latest/>`_ and `Fast CDR <https://www.eprosima.com/images/PDFs/Fast_CDR.pdf>`_ releases and the RMW needed to benefit from the latest features included in those releases.
No additional developer tools are included with this basic installation.

.. note::

    :ref:`tutorials_core_ros2_tutorials` showcase the specific features that *Vulcanexus* implements but are not yet included within the *ROS 2* ecosystem.

.. note::

    The binary package is called following the next convention ``vulcanexus-<version_name>-core`` being the ``version_name`` the corresponding adjective associated with the :ref:`specific release <release_notes>`.
    For example, for Vulcanexus v2, ``version_name`` is ``humble``.
    Consequently, in order to install this package run the following command after setting up the repository as explained in the :ref:`installation guide <linux_binary_installation>`

        .. code-block:: bash

            sudo apt install vulcanexus-humble-core

Vulcanexus Tools
----------------

**Vulcanexus Tools** is focused on helping developers with the introspection and debugging of their system.
Thus, the `eProsima Fast DDS Monitor <https://fast-dds-monitor.readthedocs.io/en/latest/>`_ and its dependency (`eProsima Fast DDS Statistics Backend <https://fast-dds-statistics-backend.readthedocs.io/en/latest/>`_) are included in order to monitor and diagnose the DDS communication network.
Also, `eProsima Shapes Demo <https://eprosima-shapes-demo.readthedocs.io/en/latest/index.html>`_ is provided as a simple demo to help testing that the communication is successfully established in the system.

.. note::

    :ref:`tutorials_tools_tools_tutorials` introduce these developer tools with some simple use case in order to show its potential.

.. note::

    The binary package is called following the next convention ``vulcanexus-<version_name>-tools`` being the ``version_name`` the corresponding adjective associated with the :ref:`specific release <release_notes>`.
    For example, for Vulcanexus v2, ``version_name`` is ``humble``.
    Consequently, in order to install this package run the following command after setting up the repository as explained in the :ref:`installation guide <linux_binary_installation>`

        .. code-block:: bash

            sudo apt install vulcanexus-humble-tools

Vulcanexus Micro
----------------

**Vulcanexus Micro** installs Micro-ROS toolkit oriented to the development and deployment of embedded *ROS 2* applications.
Thus, `eProsima Micro XRCE-DDS <https://micro-xrce-dds.docs.eprosima.com/en/latest/>`_, the protocol that enables a resource constrained, low-powered device to communicate with a DDS network through the `micro-ROS Agent <https://micro-xrce-dds.docs.eprosima.com/en/latest/agent.html>`_, and the Agent itself are installed.
Micro-ROS provides also a set of tools for integration within many build systems, toolchains and embedded development frameworks.

.. note::

    More information about Micro-ROS can be found in :ref:`tutorials_micro_micro_tutorials`.

.. note::

    The binary package is called following the next convention ``vulcanexus-<version_name>-micro`` being the ``version_name`` the corresponding adjective associated with the :ref:`specific release <release_notes>`.
    For example, for Vulcanexus v2, ``version_name`` is ``humble``.
    Consequently, in order to install this package run the following command after setting up the repository as explained in the :ref:`installation guide <linux_binary_installation>`

        .. code-block:: bash

            sudo apt install vulcanexus-humble-micro

Vulcanexus Cloud
----------------

**Vulcanexus Cloud** is oriented to geographically spaced scenarios, deploying robotic applications through a distributed network.
To this end, `eProsima DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/index.html>`_ is installed in this *Vulcanexus* package distribution, allowing DDS communication through different network layers (bridging the local networks through WAN communication).

.. note::

    :ref:`tutorials_cloud_cloud_tutorials` showcase the features and capabilities of the *Vulcanexus Cloud* package distribution.

.. note::

    The binary package is called following the next convention ``vulcanexus-<version_name>-cloud`` being the ``version_name`` the corresponding adjective associated with the :ref:`specific release <release_notes>`.
    For example, for Vulcanexus v2, ``version_name`` is ``humble``.
    Consequently, in order to install this package run the following command after setting up the repository as explained in the :ref:`installation guide <linux_binary_installation>`

        .. code-block:: bash

            sudo apt install vulcanexus-humble-cloud

Vulcanexus Simulation
---------------------

**Vulcanexus Simulation** installs the open-source robot simulator `Webots <https://cyberbotics.com/doc/guide/foreword>`_.
Webots allows to simulate real robotic environments providing examples of robot models and controllers

.. note::

    The binary package is called following the next convention ``vulcanexus-<version_name>-simulation`` being the ``version_name`` the corresponding adjective associated with the :ref:`specific release <release_notes>`.
    For example, for Vulcanexus v2, ``version_name`` is ``humble``.
    Consequently, in order to install this package run the following command after setting up the repository as explained in the :ref:`installation guide <linux_binary_installation>`

        .. code-block:: bash

            sudo apt install vulcanexus-humble-simulation

Vulcanexus Base
---------------

**Vulcanexus Base** installs the Tools, Micro and Cloud components jointly.

.. note::

    The binary package is called following the next convention ``vulcanexus-<version_name>-base`` being the ``version_name`` the corresponding adjective associated with the :ref:`specific release <release_notes>`.
    For example, for Vulcanexus v2, ``version_name`` is ``humble``.
    Consequently, in order to install this package run the following command after setting up the repository as explained in the :ref:`installation guide <linux_binary_installation>`

        .. code-block:: bash

            sudo apt install vulcanexus-humble-base

Vulcanexus Desktop
------------------

Finally, **Vulcanexus Desktop** package distribution installs all of the above packages besides `ROS 2 Desktop package distribution <https://ros.org/reps/rep-2001.html#end-user-entry-points>`_.
*ROS 2 Desktop* provides additional visualization tools, examples, demos, and tutorials.
This is the most complete *Vulcanexus* installation and it is intended for developers that want a better understanding of the *ROS 2* ecosystem.

.. note::

    The binary package is called following the next convention ``vulcanexus-<version_name>-desktop`` being the ``version_name`` the corresponding adjective associated with the :ref:`specific release <release_notes>`.
    For example, for Vulcanexus v2, ``version_name`` is ``humble``.
    Consequently, in order to install this package run the following command after setting up the repository as explained in the :ref:`installation guide <linux_binary_installation>`

        .. code-block:: bash

            sudo apt install vulcanexus-humble-desktop
