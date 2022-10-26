.. _tutorials_micro_custom_transports:

Custom Transports
=================

.. contents:: Table of Contents
    :depth: 1
    :local:
    :backlinks: none


This tutorial aims at providing step-by-step guidance for those users interested in creating micro-ROS custom transports, instead of using the ones provided by default in the micro-ROS tools set.

This tutorial starts from a previously created micro-ROS environment. Check the first steps of `First micro-ROS application on an RTOS <https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/getting_started/getting_started.html>`_ for instructions on how to create a micro-ROS environment for embedded platforms.

The micro-ROS middleware, **eProsima Micro XRCE-DDS**, provides a user API that allows interfacing with the lowest level transport layer at runtime, which enables users to implement their own transports in both the micro-ROS Client and micro-ROS Agent libraries.

Thanks to this, the Micro XRCE-DDS wire protocol can be transmitted over virtually any protocol, network or communication mechanism. In order to do so, two general communication modes are provided:

- **Stream-oriented mode**: the communication mechanism implemented does not have the concept of packet. `HDLC framing <https://micro-xrce-dds.docs.eprosima.com/en/latest/transport.html?highlight=hdlc#custom-serial-transport>`_ will be used.
- **Packet-oriented mode**: the communication mechanism implemented is able to send a whole packet that includes an XRCE message.

These two modes can be selected by activating and deactivating the ``framing`` parameter in both the micro-ROS Client and the micro-ROS Agent functions, default defines available:

.. code-block:: c

    #define MICROROS_TRANSPORTS_FRAMING_MODE 1
    #define MICROROS_TRANSPORTS_PACKET_MODE 0


micro-ROS Client
----------------

A full example can be found on micro-ROS demos repository `custom_transports <https://github.com/micro-ROS/micro-ROS-demos/blob/humble/rclc/configuration_example/custom_transports/main.c>`_ example.

An example on how to set these external transport callbacks in the micro-ROS Client API is:

.. code-block:: c

    #include <rmw_microros/rmw_microros.h>

    ...

    struct custom_args {
        ...
    }


    struct custom_args args;

    rmw_uros_set_custom_transport(
        MICROROS_TRANSPORTS_FRAMING_MODE, // Framing enabled here. Using Stream-oriented mode.
        (void *) &args,
        my_custom_transport_open,
        my_custom_transport_close,
        my_custom_transport_write,
        my_custom_transport_read
    );


It is important to notice that in ``rmw_uros_set_custom_transport`` a pointer to custom arguments is set. This reference will be available on every transport callbacks.



In general, four functions must be implemented. The behaviour of these functions is slightly different, depending on the selected mode, in all of them ``transport->args`` holds the arguments passed through ``rmw_uros_set_custom_transport``:

Open function
^^^^^^^^^^^^^

.. code-block:: c

    bool my_custom_transport_open(uxrCustomTransport* transport)
    {
        ...
    }


This function should open and init the custom transport. It returns a boolean indicating if the opening was successful.

Close function
^^^^^^^^^^^^^^

.. code-block:: c

    bool my_custom_transport_close(uxrCustomTransport* transport)
    {
        ...
    }

This function should close the custom transport. It returns a boolean indicating if closing was successful.

Write function
^^^^^^^^^^^^^^

.. code-block:: c

    size_t my_custom_transport_write(
        uxrCustomTransport* transport,
        const uint8_t* buffer,
        size_t length,
        uint8_t* errcode)
    {
        ...
    }

This function should write data to the custom transport. It returns the number of bytes written.

- **Stream-oriented mode:** The function can send up to ``length`` bytes from ``buffer``.
- **Packet-oriented mode:** The function should send ``length`` bytes from ``buffer``. If less than ``length`` bytes are written, ``errcode`` can be set.

Read function
^^^^^^^^^^^^^

.. code-block:: c

    size_t my_custom_transport_read(
        uxrCustomTransport* transport,
        uint8_t* buffer,
        size_t length,
        int timeout,
        uint8_t* errcode)
    {
        ...
    }

This function should read data from the custom transport. It returns the number of bytes read.

``transport->args`` have the arguments passed through ``rmw_uros_set_custom_transport``.

- **Stream-oriented mode:** The function should retrieve up to ``length`` bytes from the transport and write them into ``buffer`` in ``timeout`` milliseconds.
- **Packet-oriented mode:** The function should retrieve ``length`` bytes from transport and write them into ``buffer`` in ``timeout`` milliseconds. If less than ``length`` bytes are read, ``errcode`` can be set.

micro-ROS Agent
---------------

A full example can be found on Micro-XRCE-DDS-Agent repository `custom_agent <https://github.com/eProsima/Micro-XRCE-DDS-Agent/blob/master/examples/custom_agent/custom_agent.cpp>`_  example.

The micro-ROS Agent profile for custom transports is enabled by default.

An example on how to set the external transport callbacks in the micro-ROS Agent API is:

.. code-block:: c

    eprosima::uxr::Middleware::Kind mw_kind(eprosima::uxr::Middleware::Kind::FASTDDS);
    eprosima::uxr::CustomEndPoint custom_endpoint;

    // Add transport endpoing parameters
    custom_endpoint.add_member<uint32_t>("param1");
    custom_endpoint.add_member<uint16_t>("param2");
    custom_endpoint.add_member<std::string>("param3");

    eprosima::uxr::CustomAgent custom_agent(
        "my_custom_transport",
        &custom_endpoint,
        mw_kind,
        true, // Framing enabled here. Using Stream-oriented mode.
        my_custom_transport_open,
        my_custom_transport_close,
        my_custom_transport_write
        my_custom_transport_read);

    custom_agent.start();

As in the *Client* API, four functions should be implemented. The behavior of these functions is sightly different depending on the selected mode.

CustomEndPoint
^^^^^^^^^^^^^^

The ``custom_endpoint`` is an object of type ``eprosima::uxr::CustomEndPoint`` and it is in charge of handling the endpoint parameters. The **Agent**, unlike the **Client**, can receive messages from multiple **Clients** so it must be able to differentiate between them.

Therefore, the ``eprosima::uxr::CustomEndPoint`` should be provided with information about the origin of the message in the read callback, and with information about the destination of the message in the write callback.

In general, the members of a ``eprosima::uxr::CustomEndPoint`` object can be unsigned integers and strings.

``CustomEndPoint`` defines three methods:

**Add member**

.. code-block:: c

    bool eprosima::uxr::CustomEndPoint::add_member<*KIND*>(const std::string& member_name);

This function allows to dynamically add a new member to the endpoint definition.

Ir returns ``true`` if the member was correctly added, ``false`` if something went wrong (for example, if the member already exists).

- **KIND**: To be chosen from: ``uint8_t``, ``uint16_t``, ``uint32_t``, ``uint64_t``, ``uint128_t`` or ``std::string``.
- **member_name**: The tag used to identify the endpoint member.

**Set member value**

.. code-block:: c

    void eprosima::uxr::CustomEndPoint::set_member_value(const std::string& member_name, const *KIND* & value);


This function sets the specific value (numeric or string) for a certain member, which must previously exist in the ``CustomEndPoint``.

- **member_name**: The member whose value is going to be modified.
- **value**: The value to be set, of ``KIND``: ``uint8_t``, ``uint16_t``, ``uint32_t``, ``uint64_t``, ``uint128_t`` or ``std::string``.

**Get member**

.. code-block:: c

    const *KIND* & eprosima::uxr::CustomEndPoint::get_member(const std::string& member_name);


This function gets the current value of the member registered with the given parameter.

The retrieved value might be an ``uint8_t``, ``uint16_t``, ``uint32_t``, ``uint64_t``, ``uint128_t`` or ``std::string``.

- **member_name**: The `CustomEndPoint` member name whose current value is requested.

Open function
^^^^^^^^^^^^^

.. code-block:: c

    eprosima::uxr::CustomAgent::InitFunction my_custom_transport_open = [&]() -> bool
    {
        ...
    }

This function should open and init the custom transport. It returns a boolean indicating if the opening was successful.

Close function
^^^^^^^^^^^^^^

.. code-block:: c

    eprosima::uxr::CustomAgent::FiniFunction my_custom_transport_close = [&]() -> bool
    {
        ...
    }

This function should close the custom transport. It returns a boolean indicating if the closing was successful.

Write function
^^^^^^^^^^^^^^

.. code-block:: c

    eprosima::uxr::CustomAgent::SendMsgFunction my_custom_transport_write = [&](
        const eprosima::uxr::CustomEndPoint* destination_endpoint,
        uint8_t* buffer,
        size_t length,
        eprosima::uxr::TransportRc& transport_rc) -> ssize_t
    {
        ...
    }

This function should write data to the custom transport, must use the ``destination_endpoint`` members to set the data destination, returns the number of bytes written and set ``transport_rc`` indicating the result of the operation.

- **Stream-oriented mode:** The function can send up to ``length`` bytes from ``buffer``.
- **Packet-oriented mode:** The function should send ``length`` bytes from ``buffer``. If less than ``length`` bytes are written, ``transport_rc`` can be set.

Read function
^^^^^^^^^^^^^

.. code-block:: c

    eprosima::uxr::CustomAgent::RecvMsgFunction my_custom_transport_read = [&](
        eprosima::uxr::CustomEndPoint* source_endpoint,
        uint8_t* buffer,
        size_t length,
        int timeout,
        eprosima::uxr::TransportRc& transport_rc) -> ssize_t
    {
        ...
    }

This function should read data to the custom transport, must fill ``source_endpoint`` members with data source, returns the number of bytes read and set ``transport_rc`` indicating the result of the operation.

- **Stream-oriented mode:** The function should retrieve up to ``length`` bytes from the transport and write them into ``buffer`` in ``timeout`` milliseconds.
- **Packet-oriented mode:** The function should retrieve ``length`` bytes from the transport and write them into ``buffer`` in ``timeout`` milliseconds. If less than ``length`` bytes are read, ``transport_rc`` can be set.
