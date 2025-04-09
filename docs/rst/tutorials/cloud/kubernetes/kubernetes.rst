.. include:: ../../../exports/alias.include

.. _tutorials_cloud_kubernetes_kubernetes:


Vulcanexus Cloud and Kubernetes
===============================

Background
----------

This walk-through tutorial sets up both a *Kubernetes* (*K8s*) network and a local environment in order to establish communication between a pair of ROS nodes, one sending messages from a LAN (talker) and another one receiving them in the Cloud (listener).
Cloud environments such as container-oriented platforms can be connected using `eProsima DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/>`_, and thus, by launching a *DDS Router* instance at each side, communication can be established.

.. figure:: /rst/figures/tutorials/cloud/ddsrouter_overview_wan.png

Prerequisites
-------------

Ensure that the Vulcanexus installation includes the cloud and the ROS 2 demo nodes package (it is suggested to use ``vulcanexus-jazzy-desktop``).
Also, remember to source the environment in every terminal in this tutorial.

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash

.. warning::

    For the full understanding of this tutorial basic understanding of Kubernetes is required.

Local setup
-----------

The local instance of *DDS Router* (local router) only requires to have a `Simple Participant <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/participants/simple.html>`_ and a `WAN Participant <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/participants/wan.html>`_ that will play the client role in the discovery process of remote participants (see |InitialPeersFastDdsDocs|).

After having acknowledged each other's existence through `Simple DDS discovery mechanism <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html>`_ (multicast communication), the local participant will start receiving messages published by the ROS 2 talker node, and will then forward them to the WAN participant.
Next, these messages will be sent to another participant hosted on a *K8s* cluster to which it connects via WAN communication over UDP/IP.
Following there is a representation of the above-described scenario:

.. figure:: /rst/figures/tutorials/cloud/vulcanexus_local.png


Local router
^^^^^^^^^^^^

The configuration file used by the local router will be the following:

.. literalinclude:: /resources/tutorials/cloud/kubernetes/local-ddsrouter.yaml
    :language: yaml

Please, copy the previous configuration snippet and save it to a file in your current working directory with name ``local-ddsrouter.yaml``.

Note that the simple participant will be receiving messages sent in DDS domain ``0``.
Also note that, due to the choice of UDP as transport protocol, a listening address with the LAN public IP address needs to be specified for the local WAN participant, even when behaving as client in the participant discovery process.
Make sure that the given port is reachable from outside this local network by properly configuring port forwarding in your Internet router device.
The connection address points to the remote WAN participant deployed in the *K8s* cluster.
For further details on how to configure WAN communication, please have a look at `WAN Configuration <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/wan_configuration.html>`_.

.. note::

    As an alternative, `TCP transport <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/wan_configuration.html#tcp-example>`_ may be used instead of UDP.
    This has the advantage of not requiring to set a listening address in the local router's WAN participant (TCP client), so there is no need to fiddle with the configuration of your Internet router device.

To launch the local router, execute the following command (remember to source the Vulcanexus environment):

.. code-block:: bash

    ddsrouter --config-path local-ddsrouter.yaml


Talker
^^^^^^

In another terminal, run the following command in order to start the ROS 2 node that publishes messages in DDS domain ``0`` (remember to source the Vulcanexus environment):

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

Kubernetes setup
----------------

Two different deployments are required to receive the ``talker`` messages in the Cloud, each in a different *K8s* pod; the first one being a *DDS Router* cloud instance (cloud router), which consists of two participants:

* A `WAN Participant <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/participants/wan.html>`_ that receives the messages coming from our LAN through the aforementioned UDP communication channel.
* A `Local Discovery Server <https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/participants/local_discovery_server.html>`_ (local DS) that propagates them to a ROS 2 listener node hosted in a different *K8s* pod.

.. note::

    The choice of a Local Discovery Server instead of a Simple Participant to communicate with the listener has to do with the difficulty of enabling multicast routing in cloud environments.

The other deployment is the ROS 2 listener node.
This node has to be launched as a Client to the local DS running on the first deployment.

The described scheme is represented in the following figure:

.. figure:: /rst/figures/tutorials/cloud/vulcanexus_cloud.png

In addition to the two mentioned deployments, two *K8s* `services <https://kubernetes.io/docs/concepts/services-networking/service/>`_ are required in order to direct dataflow to each of the pods.
A LoadBalancer will forward messages reaching the cluster to the WAN participant of the cloud router, and a ClusterIP service will be in charge of delivering messages from the local DS to the listener pod.
Following there are the settings needed to launch these services in *K8s*:

.. literalinclude:: /resources/tutorials/cloud/kubernetes/load-balancer-service.yaml
    :language: yaml

.. literalinclude:: /resources/tutorials/cloud/kubernetes/local-service.yaml
    :language: yaml

.. note::

    An `Ingress <https://kubernetes.io/docs/concepts/services-networking/ingress/>`_ needs to be configured for the LoadBalancer service to make it externally-reachable.
    In this example we consider the assigned public IP address to be ``2.2.2.2``.

The configuration file used for the cloud router will be provided by setting up a `ConfigMap <https://kubernetes.io/docs/concepts/configuration/configmap/>`_:

.. literalinclude:: /resources/tutorials/cloud/kubernetes/ConfigMap.yaml
    :language: yaml

Following there is a representation of the overall *K8s* cluster configuration:

.. figure:: /rst/figures/tutorials/cloud/vulcanexus_k8s.png


DDS-Router deployment
^^^^^^^^^^^^^^^^^^^^^

The cloud router is launched from within a *Vulcanexus Cloud* Docker image (that can be downloaded in `Vulcanexus webpage <https://vulcanexus.org/>`_), which uses as configuration file the one hosted in the previously set up ConfigMap.
Assuming the name of the generated Docker image is ``ubuntu-vulcanexus-cloud:jazzy``, the cloud router will then be deployed with the following settings:

.. literalinclude:: /resources/tutorials/cloud/kubernetes/ddsrouter.yaml
    :language: yaml


Listener deployment
^^^^^^^^^^^^^^^^^^^

Since ROS 2 demo nodes package is not installed by default in *Vulcanexus Cloud*, a new Docker image adding in this functionality must be generated.
Also, the IP address and port of the local Discovery Server must be specified, so a custom entrypoint is also provided.

Copy the following snippet and save it to the current directory as ``Dockerfile``:

.. literalinclude:: /resources/tutorials/cloud/kubernetes/Dockerfile_cloud
    :language: Dockerfile

Copy the following snippet and save it to the current directory as ``run.bash``:

.. literalinclude:: /resources/tutorials/cloud/kubernetes/run.bash
    :language: bash

Build the docker image running the following command:

.. code-block:: bash

    docker build -t vulcanexus-cloud-demo-nodes:jazzy -f Dockerfile

Now, the listener pod can be deployed by providing the following configuration:

.. literalinclude:: /resources/tutorials/cloud/kubernetes/listener.yaml
    :language: yaml

Once all these components are up and running, communication should have been established between the talker and listener nodes, so that messages finally manage to reach the listener pod and get printed in its ``STDOUT``.

Feel free to interchange the locations of the ROS nodes by slightly modifying the provided configuration files, hosting
the talker in the *K8s* cluster while the listener runs in the LAN.
