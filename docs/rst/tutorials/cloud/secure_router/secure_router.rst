.. include:: ../../../exports/alias.include
.. _tutorials_ddsrouter_participant_via_xml_profiles:

Secure communications with ROS 2 Router
=======================================

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

Preparing all security certificates
-----------------------------------

Participants security can only be configured by XML, following the instructions of `Fast DDS Security Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/security.html>`_ to fulfill the pertinent requirements of the chosen security mechanisms.
To generate the security certificates needed for both *talker* and *listener* nodes and both |rosrouter| participants please follow the `Setting up security <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Advanced/Security/Introducing-ros2-security.html>`_ ROS 2 tutorial.
Additionally to the steps followed in the tutorial, when generating keys and certificates for the *talker* and *listener* nodes (`Section 3 <https://docs.vulcanexus.org/en/latest/ros2_documentation/source/Tutorials/Advanced/Security/Introducing-ros2-security.html#generate-keys-and-certificates>`__), generate also their corresponding security files for both |rosrouter| participants as follows:

.. code-block:: bash

    ros2 security create_enclave demo_keystore /ros2_router/participant0
    ros2 security create_enclave demo_keystore /ros2_router/participant1

Configuring *governance.xml* and *permissions.xml*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default, the ``create_enclave`` feature configures security for ROS Domain ``0``.
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

For our specific case, we will modify the corresponding *listener* and ROS 2 Router participant in domain 1 *permissions.xml* files to apply to ROS 2 Domain ``1``.
In order to do so, please find the ``<domain></domain>`` section in

* ``~/sros2_demo/demo_keystore/enclaves/talker_listener/listener/permissions.xml``, and
* ``~/sros2_demo/demo_keystore/enclaves/ros2_router/participant1/permissions.xml``

files and replace it by:

.. code-block:: xml

    <domains>
        <id>1</id>
    </domains>

We also need to update the *governance.xml* file (``~/sros2_demo/demo_keystore/enclaves/governance.xml``) that applies to all entities in our demo to include both ROS 2 Domains ``0`` and ``1``.

.. code-block:: xml

    <domains>
        <id_range>
            <min>0</min>
            <max>1</max>
        </id_range>
    </domains>

.. note::

    Make sure that you have applied the changes in all ``<domain></domain>`` sections, as some files have it more than once.

Signing *governance.xml* and *permissions.xml*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once applied the changes, the updated files must be signed with the permission CA certificate.

*   Sign permissions files

    .. code-block:: bash

        openssl smime -sign -text \
            -in ~/sros2_demo/demo_keystore/enclaves/talker_listener/listener/permissions.xml \
            -out ~/sros2_demo/demo_keystore/enclaves/talker_listener/listener/permissions.p7s \
            -signer ~/sros2_demo/demo_keystore/public/permissions_ca.cert.pem \
            -inkey ~/sros2_demo/demo_keystore/private/permissions_ca.key.pem

        openssl smime -sign -text \
            -in ~/sros2_demo/demo_keystore/enclaves/ros2_router/participant1/permissions.xml \
            -out ~/sros2_demo/demo_keystore/enclaves/ros2_router/participant1/permissions.p7s \
            -signer ~/sros2_demo/demo_keystore/public/permissions_ca.cert.pem \
            -inkey ~/sros2_demo/demo_keystore/private/permissions_ca.key.pem

*   Sign governance file

    .. code-block:: bash

        openssl smime -sign -text -in ~/sros2_demo/demo_keystore/enclaves/governance.xml \
            -text -out ~/sros2_demo/demo_keystore/enclaves/governance.p7s \
            -signer ~/sros2_demo/demo_keystore/public/permissions_ca.cert.pem \
            -inkey ~/sros2_demo/demo_keystore/private/permissions_ca.key.pem


Configuration of ROS 2 Router
-----------------------------

The following YAML configuration file configures a |rosrouter| to create two participants in ROS 2 Domains ``0`` and ``1`` so that it creates a bridge between the isolated networks enabling communication between them.

.. literalinclude:: /resources/tutorials/cloud/router_participant_security_via_xml_profiles/ddsrouter.yaml
    :language: yaml

Please, save this file in ``~/sros_demo/ros2_router`` directory.

ROS 2 Router Participants Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A `ROS 2 Router Participant <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/participants/participant.html>`_ is a |rosrouter| entity that works as an interface between a network and the core of the router.
Participants security can only be configured via XML profiles, following the instructions of `Fast DDS Security Documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/security.html>`_ to fulfill the pertinent requirements of the chosen security mechanisms.

.. tabs::

    .. tab:: Participant Domain 0

        .. literalinclude:: /resources/tutorials/cloud/router_participant_security_via_xml_profiles/configurations/secure_domain0.xml
            :language: xml

    .. tab:: Participant Domain 1

        .. literalinclude:: /resources/tutorials/cloud/router_participant_security_via_xml_profiles/configurations/secure_domain1.xml
            :language: xml

Please add these files (*secure_domain_0.xml* and *secure_domain_1.xml*) to the current workspace so they can be referenced when running the ROS 2 Router.
At this point we have a workspace structure as follows:



These XML snippets configure security settings for two participants in the Fast DDS middleware.
They activate the PKI-DH plugin, specifying the paths to the participants' identities certificates, identity CA certificate, and private keys for authentication through the `DDS:Auth:PKI-DH plugin <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/auth_plugin/auth_plugin.html>`_.
Additionally, they enable access control through the `DDS:Access:Permissions plugin <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/access_control_plugin/access_control_plugin.html>`_, defining paths to the permissions CA certificate, governance files, and permissions files.
The configuration also includes the activation of the `DDS:Crypto:AES-GCM-GMAC <https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/crypto_plugin/crypto_plugin.html>`_ plugin for data encryption, enhancing communication security.

Running the tutorial
--------------------

Once all the files needed have been generated, the workspace ``sros2_demo`` generated in the previous steps will have the following structure:

.. code-block:: bash

    ~/sros2_demo
    ├── ros2_router
    │   ├── secure_domain_0.xml
    │   ├── secure_domain_1.xml
    │   ├── ddsrouter.yaml
    └── demo_keystore
        ├── enclaves
        │   ├── ros2_router
        │   ├── talker_listener
        │   ├── governance.p7s
        │   ├── governance.xml
        ├── private
        └── public

Running ROS2 nodes with security
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Now that everything is prepared, we can launch the demo nodes with security.
For this, open two different terminals and setup the Vulcanexus environment and the corresponding environment variables for configuring ROS 2 secure communications.

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash
    export ROS_SECURITY_KEYSTORE=<absolute/path/to/sros2_demo>/sros2_demo/demo_keystore
    export ROS_SECURITY_ENABLE=true
    export ROS_SECURITY_STRATEGY=Enforce

Open the first terminal to run a ROS 2 ``demo_nodes_cpp`` *talker* node in domain ``0`` with its corresponding security configuration:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

And the same with the *listener* in domain ``1`` on a second terminal:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener --ros-args --enclave /talker_listener/listener

At this point, the *listener* should not receive any data from the *talker* since they are in different domains. To connect them, we will use the |rosrouter|.

Running DDS Router
^^^^^^^^^^^^^^^^^^

Open a third terminal and source the Vulcanexus environment:

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash

Then run the |rosrouter| with the *yaml* configuration file created before.

.. code-block:: bash

    ddsrouter --config-path ~/sros2_demo/ros2_router/ddsrouter.yaml

The output from the |rosrouter| should be something like:

.. code-block:: bash

    Starting DDS Router Tool execution.
    DDS Router running.

If so, the |rosrouter| has started correctly and it is currently running.
Once the |rosrouter| is running, it will forward the messages from the *talker* on domain 0 to the *listener* on domain 1.
In order to close the execution, press ^C or send a signal (:code:`SIGINT 2` or :code:`SIGTERM 15`) to stop it.

To make sure security is working properly try running again the *talker* and *listener* ROS 2 nodes the security configuration:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker


.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener

This way the |rosrouter| will not receive any data from the *talker* and therefore neither will the *listener*.
