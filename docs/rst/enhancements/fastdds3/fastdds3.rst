.. _fastdds3:

Support for Fast DDS v3
=======================

Vulcanexus now includes support for the new `Fast DDS major version 3 <https://fast-dds.docs.eprosima.com/en/v3.0.0/notes/notes.html>`__, the latest version of the Data Distribution Service (DDS) implementation from eProsima.
Fast DDS *v3.0.0* introduces a range of improvements that align with Vulcanexus's mission to provide seamless and efficient middleware solutions for robotics, automotive, aerospace, and other high-demand industries.

Most relevant features
^^^^^^^^^^^^^^^^^^^^^^

  Fast DDS *v3.0.0* comes with significant changes and improvements, such as:

   * Dynamic Language Binding API according to OMG XTypes v1.3 specification.
   * RTPS layer APIs refactor.
   * Extended XML configurations (e.g FlowControllers).
   * Improvements on creation of RTPS messages.
   * Public API and namespace update.
   * Project renamed to ``fastdds``.
   * Refactored examples.
   * Removed deprecated fastrtps-related API.
   * Further improvements can be found in `Fast DDS v3.0.0 <https://github.com/eProsima/Fast-DDS/releases/tag/v3.0.0>`__.

Implications
^^^^^^^^^^^^

The following considerations have to be taken into account when using Vulcanexus with Fast DDS v3:

* Communicating Vulcanexus with Fast DDS native applications:

  * Update Fast DDS version to *v3.0.0* or higher.
    You can refer to our `Migration guide <https://github.com/eProsima/Fast-DDS/blob/master/UPGRADING.md>`__ for more detailed information.
  * When generating types, the minimum required version of Fast DDS-Gen is *v4.0.0*.
    If this version is used, the extensibility must be explicitly marked as ``@extensibility(FINAL)`` in idl structs.
    However, starting from Fast DDS-Gen *v4.0.1*, it can be left unspecified, as it will default to ``@final``.
    Please, refer to `Fast DDS data types extensibility <https://fast-dds.docs.eprosima.com/en/latest/fastddsgen/dataTypes/dataTypes.html#extensibility>`__ for further information about type extensibility.

* QoS settings for `Data Representation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/xtypesExtensions.html#datarepresentationqospolicy>`__ has to be set to ``XCDR_DATA_REPRESENTATION``.
