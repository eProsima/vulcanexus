.. _vulcanexus_recordreplay:

ROS 2 Record & Replay
=====================

*ROS 2 Record & Replay* (a.k.a. *DDS Record & Replay*) is an end-user software application that efficiently saves ROS 2 messages published into a DDS environment in a MCAP format database.
Thus, the exact playback of the recorded network events is possible as the data is linked to the timestamp at which the original data was published.

*ROS 2 Record & Replay* is easily configurable and installed with a default setup, so that DDS topics, data types and entities are automatically discovered without the need to specify the types of data recorded.
This is because the recording tool leverages the DynamicTypes functionality of `eProsima Fast DDS <https://fast-dds.docs.eprosima.com>`_, the C++ implementation of the `DDS (Data Distribution Service) Specification <https://www.omg.org/spec/DDS/About-DDS/>`_ defined by the `Object Management Group (OMG) <https://www.omg.org/>`_.

Overview
--------

*ROS 2 Record & Replay* includes the following tools:

* **ROS 2 Recorder tool**.
  The main functionality of this tool is to save the data in a `MCAP <https://mcap.dev/>`_ database.
  The database contains the records of the publication timestamp of the data, the serialized data, and the definition of the data serialization type and format.
  The output MCAP file can be read with any user tool compatible with MCAP file reading since it contains all the necessary information for reading and reproducing the data.

  .. figure:: /rst/figures/intro/tools/recordreplay/ddsrecord_overview.png
    :align: center

* **ROS 2 Replay tool**.
  This application allows to reproduce DDS traffic recorded with a *ROS 2 Recorder*.
  A user can specify which messages to replay by either setting a time range (begin/end times) out of which messages will be discarded, or directly by blocking/whitelisting a set of topics of interest.
  It is also possible to choose a different playback rate, as well as to use topic QoS different to the ones recorded.

  .. figure:: /rst/figures/intro/tools/recordreplay/ddsreplay_overview.png
    :align: center

Next Steps
----------

Visit `eProsima DDS Record & Replay Documentation <https://dds-recorder.readthedocs.io/en/latest>`_ for more information on how to configure and deploy a *ROS 2 Record & Replay* instance.
Also feel free to review :ref:`Vulcanexus Tools Tutorials <tutorials_tools_tools_tutorials>` to discover the possibilities that this tool has to offer in different scenarios.
