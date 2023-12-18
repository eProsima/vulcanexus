.. include:: ../../../exports/alias.include
.. _tutorials_ddsrouter_participant_via_xml_profiles:

Secure communications with ROS 2 Router
=======================================

.. contents::
    :depth: 2
    :local:
    :backlinks: none


Background
----------

*eProsima ROS 2 Router*, a.k.a `DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/rst/formalia/titlepage.html>`_, is a cross-platform non-graphical application developed by eProsima that allows users to connect physically or virtually separated DDS networks.

Configuring the |rosrouter| to have one or many interfaces for each isolated network will allow to establish the communication and data transmission between the nodes.
In compliance with the DDS Security specification, Fast DDS provides secure communication by implementing pluggable security at three levels:

* Authentication (`DDS:Auth:PKI-DH plugin <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/auth_plugin/auth_plugin.html>`_): provides the mechanisms and operations required for nodes (DomainParticipants) authentication at discovery.
* Access control  (`DDS:Access:Permissions plugin <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/access_control_plugin/access_control_plugin.html>`_): after a remote node (DomainParticipant) is authenticated, its permissions are validated and enforced.
* Data encryption (`DDS:Crypto:AES-GCM-GMAC <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/crypto_plugin/crypto_plugin.html>`_): provides the tools and operations required to support encryption and decryption, digests computation, message authentication codes computation and verification, key generation, and key exchange for nodes (DomainParticipants), publishers (DataWriters) and subscriptions (DataReaders).

Fast DDS allows security plugins to be activated through the `DomainParticipantQos <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html#dds-layer-domainparticipantqos>`_ properties.

This tutorial will show how to configure the |rosrouter| via XML profiles to setup a secure communication.
Specifically, the configured |rosrouter| will allow the communication between two distinct domains, a capability that would be unattainable in the absence of the Router.
The chosen security configuration in this case will be related to authentication, access control, and data encryption.

To know more about Security plugins, please refer to `Fast DDS Security Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/security.html>`_.

.. figure:: /rst/figures/tutorials/cloud/secure_router.png
   :align: center

This tutorial will use the ``demo_nodes_cpp`` ROS 2 package, available in the Vulcanexus Desktop distribution.
Two ROS 2 nodes, a *talker* and a *listener*, will be launched on different ROS 2 Domains, so that they cannot communicate between each other in a first instance.
Then, the |rosrouter| will be used as a bridge between the two Domains, checking the security features between the Participants and allowing the *listener* to receive the messages from the *talker*.


Prerequisites
-------------

To proceed, please install Vulcanexus with one of the following installation methods:

* :ref:`linux_binary_installation`
* :ref:`linux_source_installation`
* :ref:`docker_installation`

Creating all security certificates
-----------------------------------

Participants security can only be configured by XML, following the instructions of `Fast DDS Security Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/security.html>`_ to fulfill the pertinent requirements of the chosen security mechanisms.
To generate the security certificates needed for both *talker* and *listener* nodes and both Domain Participants follow the Vulcanexus Tutorial `Setting up security <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Advanced/Security/Introducing-ros2-security.html>`_.
Additionally to the steps followed in the tutorial, when generating keys and certificates for the *talker* and *listener* nodes, generate also their corresponding security files for both Domain Participants as follows:

.. code-block:: bash

    ros2 security create_enclave demo_keystore /participants/participant0
    ros2 security create_enclave demo_keystore /participants/participant1

Configuring *governance.xml* and *permissions.xml*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default, the ``create_enclave`` feature configures security in ``ROS_DOMAIN_ID=0``.
We can set up another domain or even a range of domains applying the following change in ``<domain></domain>`` in the *governance.xml* and in the *permissions.xml* files.

.. tabs::

    .. tab:: Change to another domain

        .. code-block:: xml

            <domains>
                <id>(new domain)</id>
            </domains>

    .. tab:: Change to a range of domains

        .. code-block:: xml

            <domains>
                <id_range>
                    <min>(minimum domain id)</min>
                    <max>(maximum domain id)</max>
                </id_range>
            </domains>

For our specific case, we will set the domain to ``ROS_DOMAIN_ID=1`` in the *listener* and *Participant 1* permissions and a range including both 0 and 1 domains in the governance file.

.. note::

    Make sure that you have applied the changes in all ``<domain></domain>`` sections, as some files have it more than once.

Signing *governance.xml* and *permissions.xml*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once applied the changes, the files must be signed with the permission CA certificate:

.. tabs::

    .. tab:: Permissions

        .. code-block:: bash

            openssl smime -sign -text -in <path/to/workspace_ddsrouter-tutorial>/demo_keystore/enclaves/talker_listener/listener/permissions.xml \
                -out <path/to/workspace_ddsrouter-tutorial>/demo_keystore/enclaves/talker_listener/listener/permissions.p7s \
                -signer <path/to/workspace_ddsrouter-tutorial>/demo_keystore/public/permissions_ca.cert.pem \
                -inkey <path/to/workspace_ddsrouter-tutorial>/demo_keystore/private/permissions_ca.key.pem
            openssl smime -sign -text -in <path/to/workspace_ddsrouter-tutorial>/demo_keystore/enclaves/participants/particpant1/permissions.xml \
                -out <path/to/workspace_ddsrouter-tutorial>/demo_keystore/enclaves/participants/particpant1/permissions.p7s \
                -signer <path/to/workspace_ddsrouter-tutorial>/demo_keystore/public/permissions_ca.cert.pem \
                -inkey <path/to/workspace_ddsrouter-tutorial>/demo_keystore/private/permissions_ca.key.pem

    .. tab:: Governance

        .. code-block:: bash

            openssl smime -sign -text -in <path/to/workspace_ddsrouter-tutorial>/demo_keystore/enclaves/governance.xml \
                -text -out <path/to/workspace_ddsrouter-tutorial>/demo_keystore/enclaves/governance.p7s \
                -signer <path/to/workspace_ddsrouter-tutorial>/demo_keystore/public/permissions_ca.cert.pem \
                -inkey <path/to/workspace_ddsrouter-tutorial>/demo_keystore/private/permissions_ca.key.pem

Configuration of ROS 2 Router
-----------------------------

The following YAML configuration file configures a |rosrouter| to create two participants in ROS 2 Domains ``0`` and ``1`` so that it creates a bridge between the isolated networks enabling communication between them.

.. literalinclude:: /resources/tutorials/cloud/router_participant_security_via_xml_profiles/ddsrouter.yaml
    :language: yaml

ROS 2 Router Participants Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

DDS Router `Participant <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/user_manual_glossary.html#term-Participant>`_ is a DDS Router entity that works as an interface between a network and the core of the router.
Participants security can only be configured by XML, following the instructions of `Fast DDS Security Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/security.html>`_ to fulfill the pertinent requirements of the chosen security mechanisms.

.. tabs::

    .. tab:: Participant Domain 0

        .. literalinclude:: /resources/tutorials/cloud/router_participant_security_via_xml_profiles/configurations/secure_configuration0.xml
            :language: xml

    .. tab:: Participant Domain 1

        .. literalinclude:: /resources/tutorials/cloud/router_participant_security_via_xml_profiles/configurations/secure_configuration1.xml
            :language: xml

These XMLs snippet configure security settings for two participants in the Fast DDS middleware, focusing on authentication through the `DDS:Auth:PKI-DH plugin <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/auth_plugin/auth_plugin.html>`_.
They activate the PKI-DH plugin, specifying the paths to the participants' identities certificates, identity CA certificate, and private keys for authentication.
Additionally, they enable access control through the `DDS:Access:Permissions plugin <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/access_control_plugin/access_control_plugin.html>`_, defining paths to the permissions CA certificate, governance files, and permissions files.
The configuration also includes the activation of the `DDS:Crypto:AES-GCM-GMAC <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/crypto_plugin/crypto_plugin.html>`_ plugin for data encryption, enhancing communication security within the Fast DDS framework.

Running the tutorial
--------------------

Once all the files needed have been generated, create a workspace called ``workspace_ddsrouter-tutorial`` to collect all the directories and files.
The tutorial workspace will have the following structure at the end of the project:

.. code-block:: bash

    workspace_ddsrouter-tutorial
    ├── configurations
    │   ├── secure_configuration0.xml
    │   ├── secure_configuration1.xml
    ├── demo_keystore
    │   ├── enclaves
    │   │   ├── participants
    │   │   ├── talker_listener
    │   │   ├── governance.p7s
    │   │   ├── governance.xml
    │   ├── private
    │   ├── public
    └── ddsrouter.yaml

Running ROS2 nodes with security
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Now that everything is prepared, we can launch the demo nodes with security.
For this, open two different terminals and run Vulcanexus Docker images on them, including the workspace created as a volume:

.. code-block:: bash

    docker run -it -v <path/to/workspace_ddsrouter-tutorial>:/home ubuntu-vulcanexus:iron-desktop

.. note::

    Remember to source /opt/vulcanexus/iron/setup.bash and export the corresponding environmental variables to set up security in all the terminals run in this tutorial., as explained in the `ROS 2 Tutorial <https://docs.ros.org/en/iron/Tutorials/Advanced/Security/Introducing-ros2-security.html>`_.

    .. code-block:: bash

        source /opt/vulcanexus/iron/setup.bash
        export ROS_SECURITY_KEYSTORE=home/demo_keystore
        export ROS_SECURITY_ENABLE=true
        export ROS_SECURITY_STRATEGY=Enforce

Open the first terminal to run a ROS 2 ``demo_nodes_cpp`` *talker* in domain ``0`` with its corresponding security:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

And the same with the *listener* in domain ``1`` on a second terminal:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener --ros-args --enclave /talker_listener/listener

At this point, the *listener* should not receive any data from the *talker* since they are in different domains. To connect them, we will use the ROS 2 Router.

Running DDS Router
^^^^^^^^^^^^^^^^^^

On a third terminal run another Docker image and launch the `DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/rst/formalia/titlepage.html>`_ with the configuration file available at ``<path/to/file>/ddsrouter.yaml``.

.. code-block:: bash

    ddsrouter --config-path <path/to/file>/ddsrouter.yaml

The output from the `DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/rst/formalia/titlepage.html>`_ should be something like:

.. code-block:: bash

    Starting DDS Router Tool execution.
    DDS Router running.

If so, the `DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/rst/formalia/titlepage.html>`_ has started correctly and it is currently running. Once the `DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/rst/formalia/titlepage.html>`_ is running, it will forward the messages from the *talker* on domain 0 to the *listener* on domain 1.
In order to close the execution, press ^C or send a signal (:code:`SIGINT 2` or :code:`SIGTERM 15`) to close it.

To make sure security is working properly try running again the nodes without the ``--ros-args`` argument such as:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker


.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener

This should not allow the *listener* to receive any data from the *talker*.
