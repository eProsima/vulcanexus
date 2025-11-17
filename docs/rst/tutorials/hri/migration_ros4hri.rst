.. _tutorials_hri_migration_ros4hri:

Migration from ROS4HRI to Vulcanexus HRI Tutorial
==================================================

Background
----------

`ROS4HRI <https://github.com/ros4hri>`_ is a framework for Human-Robot Interaction (HRI) in ROS 2 that implements the guidelines defined in `ROS REP-155 <https://ros.org/reps/rep-0155.html>`_.
The specification relies on the use of dedicated namespaces for each detected human entity.
For example, when a new face is detected, a new namespace such as ``/humans/faces/<unique_id>/`` is created.
Under this namespace, multiple topics coexist to publish complementary information:

- ``/humans/faces/<unique_id>/roi``
- ``/humans/faces/<unique_id>/cropped``
- ``/humans/faces/<unique_id>/landmarks``
- ``/humans/faces/<unique_id>/pose``
- And up to 8 topics in total for each face instance.

This design provides modularity and separation of concerns, but introduces significant complexity in terms of middleware management, discovery, and scalability.
In scenarios with multiple detected humans, the number of topics, publishers, and subscribers increases rapidly, leading to performance bottlenecks and challenges in code maintenance.

Vulcanexus HRI libraries introduce a new approach by leveraging **Fast DDS keyed topics** and message aggregation mechanisms,
which drastically reduces the number of DDS entities required to manage HRI data.
This tutorial will guide you through the process of migrating from ROS4HRI to Vulcanexus HRI packages, highlighting the key **API differences** and benefits of the new approach.

For a deeper explanation of the Vulcanexus HRI messages or a direct comparison on a practical use case, please refer to the :ref:`hri_msgs` section.

Key Differences between ROS4HRI and Vulcanexus HRI APIs
-------------------------------------------------------

The primary differences between ROS4HRI and Vulcanexus HRI packages are summarized below:

- **Namespace vs Keyed Topics**: ROS4HRI creates a dedicated namespace and multiple topics for each detected human, while Vulcanexus HRI uses **keyed topics** to publish all entities of the same type on a single topic.
- **Number of DDS Entities**: Vulcanexus HRI significantly reduces the number of DDS entities by using a single publisher and subscriber per topic, regardless of the number of detected humans.
- **Message Aggregation**: Vulcanexus HRI supports message aggregation, allowing multiple instances of the same entity type to be published in a single message, further reducing the overhead of DDS communication.
- **Reduced Complexity**: The simplified architecture of Vulcanexus HRI leads to **easier code maintenance**, **improved performance**, and **scalability** in scenarios with multiple detected humans.
- **Simplified Discovery**: With fewer DDS entities, the discovery process is more efficient, leading to **reduced network traffic** and **lower resource consumption**.
- **ROS 2 Kilted and Jazzy Support**: Vulcanexus HRI packages are designed to be compatible with the latest ROS 2 distributions, ensuring long-term support and access to new features.

Besides these architectural differences, the APIs of the Vulcanexus HRI packages have been designed to replicate ROS4HRI APIs as much as possible, minimizing the effort required for migration.

In particular, ROS4HRI relies on the package `libhri <https://github.com/ros4hri/libhri>`_ for implementing a wrapper library that simplifies the interaction with HRI data.
Vulcanexus HRI packages provide a mirror wrapper library with comparable APIs, making the migration process almost seamless.

Wrapping Libraries Comparison
-----------------------------

Both ROS4HRI and Vulcanexus HRI wrapping libraries provide the same public APIs.
This means that any code written using the ROS4HRI wrapping library is directly API-compatible with the Vulcanexus HRI wrapping library, requiring only changes in the sourced workspace.

.. warning::

    Both packages implement the same library and class names to ensure API compatibility.
    Therefore, ensure that your ROS 2 workspace sources the correct setup file to avoid conflicts between the two libraries.
    Never compile both libraries in the same workspace.

The import statements for both libraries are identical and do not require any changes.
Similarly, the creation of HRI listener objects and access to detected faces or bodies remains unchanged.

.. tab-set::
  .. tab-item:: Python
    :sync: python

    .. code-block:: python

      # ROS4HRI import AND Vulcanexus HRI import have the same statement
      from hri import HRIListener

      # Creating HRI listener in ROS4HRI AND Vulcanexus HRI is identical
      class ExampleNode(Node):
        def __init__(self):
          super().__init__('pyhri_example_node')
          self.hri_listener = HRIListener('pyhri_example_hri_listener')

        def access_data(self):
          # Accessing faces/bodies in ROS4HRI AND Vulcanexus HRI is identical
          for face_id, face in self.hri_listener.faces.items():
            print(f"Face ID: {face_id}")

          for body_id, body in self.hri_listener.bodies.items():
            print(f"Body ID: {body_id}")

  .. tab-item:: C++
    :sync: cpp

    .. code-block:: cpp

      // ROS4HRI import AND Vulcanexus HRI import have the same statement
      #include "hri/hri.hpp"

      // Creating HRI listener in ROS4HRI AND Vulcanexus HRI is identical
      class ShowFaces : public rclcpp::Node
      {
      public:
        void init()
        {
          hri_listener_ = hri::HRIListener::create(shared_from_this());
        }

        void access_data()
        {
          // Accessing faces/bodies in ROS4HRI AND Vulcanexus HRI is identical
          auto faces = hri_listener_->getFaces();
          for (auto const& [face_id, face] : faces)
          {
            RCLCPP_INFO(this->get_logger(), "Face ID: %s", face_id.c_str());
          }

          auto bodies = hri_listener_->getBodies();
          for (auto const& [body_id, body] : bodies)
          {
            RCLCPP_INFO(this->get_logger(), "Body ID: %s", body_id.c_str());
          }
        }
      private:
        std::shared_ptr<hri::HRIListener> hri_listener_;
      };

Data under each face/body instance is also accessed using the same APIs, although not all data fields are available in both libraries.
For example, Vulcanexus HRI library do not publish cropped or aligned images for faces or bodies to reduce bandwidth usage.
In this case, the corresponding APIs will return ``None`` (Python) or ``std::nullopt`` (C++).
As the API signatures are identical and each method always returns an ``std::optional``, existing code will still compile and run without modifications.
Just ensure that your code correctly handles the absence of data when using these APIs and never attempt to access the cropped/aligned images without checking for their existence first.

.. tab-set::
  .. tab-item:: Python
    :sync: python

    .. code-block:: python

      for face_id, face in self.hri_listener.faces.items():
        if (face.cropped is not None):
          # Cropped image is not sent to avoid network congestion
          # This code will never be reached when using Vulcanexus HRI
          pass

  .. tab-item:: C++
    :sync: cpp

    .. code-block:: cpp

      auto faces = hri_listener_->getFaces();
      for (auto const& [face_id, face] : faces)
      {
        if (auto cropped = face->cropped())
        {
          // Cropped image is not sent to avoid network congestion
          // This code will never be reached when using Vulcanexus HRI
        }
      }

However, cropped images are still available by subscribing directly to the corresponding image topic and using the Region of Interest (ROI) data provided for each face or body.
An example about how to do this is available in the Vulcanexus HRI API library repository for both `Python <https://github.com/eProsima/agile-hri/blob/main/hri_api/vulcanexus_hri_py/hri/example.py>`_ and `C++ <https://github.com/eProsima/agile-hri/tree/main/hri_api/vulcanexus_hri_cpp/src>`_.

New Features in Vulcanexus HRI
-------------------------------

Vulcanexus HRI packages introduce several new features that were not available in ROS4HRI, like the Speech-to-Text (STT) and Text-to-Speech (TTS) capabilities.

Both functionalities are implemented in dedicated packages, ``hri_stt`` and ``hri_tts`` respectively, and provide ROS2 action based nodes.
Vulcanexus HRI API library extends the wrapping library to include STT features through the ``HRIListener`` method ``getSpeech()``.

.. tab-set::
  .. tab-item:: Python
    :sync: python

    .. code-block:: python

      from hri import HRIListener
      import rclpy
      class RequestSpeech():
        def __init__(self):
          # In Python wrapper there is no need to initialize the HRIListener within a ROS 2 Node class
          # The HRIListener creates its own node internally and handles the spinning automatically
          self.hri_listener = HRIListener('vulcanexus_hri_py_example_hri_listener')

        def request_speech(self) -> str:
          """Request speech from the user and return the recognized text."""
          recognized_text = None
          try:
            recognized_text = self.hri_listener.speech()
          except Exception as e:
            print(f'Error while requesting speech: {e}')
          if recognized_text is None:
            print('No speech recognized.')
            return ''
          return recognized_text

        def main(args=None):
          rclpy.init(args=args)
          rs = RequestSpeech()
          try:
            speech = rs.request_speech()
            print(f'Recognized speech: {speech}')
          except KeyboardInterrupt:
            print('Shutting down voice example node.')
          except Exception as e:
            print(f'Error in voice example node: {e}')
          finally:
            rclpy.shutdown()

  .. tab-item:: C++
    :sync: cpp

    .. code-block:: cpp

      #include <functional>
      #include <memory>
      #include "rclcpp/rclcpp.hpp"
      #include "hri/hri.hpp"
      class RequestSpeech : public rclcpp::Node
      {
      public:

        RequestSpeech()
          : rclcpp::Node("hri_cpp_example_face"){}

        void init()
        {
          // "shared_from_this()" cannot be used in the constructor!
          hri_listener_ = hri::HRIListener::create(shared_from_this());
        }

        std::string request_speech()
        {
          std::string speech;
          try
          {
            RCLCPP_INFO(this->get_logger(), "Requesting speech from user...");
            speech = hri_listener_->getSpeech();
          }
          catch (const std::exception& e)
          {
            RCLCPP_ERROR(this->get_logger(), "Error requesting speech: %s", e.what());
            throw;
          }
          RCLCPP_INFO(this->get_logger(), "Recognized speech: %s", speech.c_str());
          return speech;
        }

      private:
        std::shared_ptr<hri::HRIListener> hri_listener_;
      };

      int main(
        int argc,
        char* argv[])
      {
        rclcpp::init(argc, argv);

        auto node = std::make_shared<RequestSpeech>();
        node->init();
        try
        {
          std::string speech = node->request_speech();
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(node->get_logger(), "Error requesting speech: %s", e.what());
        }

        return 0;
      }

.. note::

    The ``getSpeech()`` method is only available in the Vulcanexus HRI API library and has no equivalent in ROS4HRI.

For more details or options about the STT and TTS packages, please refer to the respective documentation sections: :ref:`tutorials_hri_stt` and :ref:`tutorials_hri_tts`.

Summary
-------

The following tables summarizes the key differences between ROS4HRI and Vulcanexus HRI, focusing on which methods or data fields are available in each library.
C++ API is used as reference, but the same applies to Python API as well.
Note that the table only illustrates the differences and similarities about the data published, but Vulcanexus HRI API is fully compatible with ROS4HRI API for all methods, meaning that all methods are exposed in both libraries.
Any methods marked as "Not Available" in the table exist and may still be called the API, but they do not have any effect or no data is returned.

HRIListener API
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :class: longtable

   * - Methods
     - Vulcanexus HRI
     - ROS4HRI
   * - **create**
     - Available
     - Available
   * - **getFaces**
     - Available
     - Available
   * - **onFace**
     - Available
     - Available
   * - **onFaceLost**
     - Available
     - Available
   * - **getBodies**
     - Available
     - Available
   * - **onBody**
     - Available
     - Available
   * - **onBodyLost**
     - Available
     - Available
   * - **getSpeech**
     - Available
     - **NOT Implemented**
   * - **set_stt_timeouts**
     - Available
     - **NOT Implemented**
   * - **onVoice**
     - Replaced by HRI_STT with `getSpeech()`
     - **Not Available**
   * - **onVoiceLost**
     - Replaced by HRI_STT with `getSpeech()`
     - **Not Available**
   * - **getPersons**
     - Implicit in the keyed topics
     - Available
   * - **onPerson**
     - **Not Available**
     - Available
   * - **onPersonLost**
     - **Not Available**
     - Available
   * - **getTrackedPersons**
     - Implicit in the keyed topics
     - Available
   * - **onTrackedPerson**
     - **Not Available**
     - Available
   * - **onTrackedPersonLost**
     - **Not Available**
     - Available

Face Data API
^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :class: longtable

   * - Methods / Data Fields
     - Vulcanexus HRI
     - ROS4HRI
   * - **roi**
     - Available
     - Available
   * - **facialLandmarks**
     - Available
     - Available
   * - **expression**
     - Available
     - Available
   * - **cropped**
     - **Not Available**
     - Available
   * - **aligned**
     - **Not Available**
     - Available
   * - **facialActionUnits**
     - **Not Available**
     - Available
   * - **age**
     - **Not Available**
     - **Not Available**
   * - **gender**
     - **Not Available**
     - **Not Available**
   * - **expressionVA**
     - **Not Available**
     - **Not Available**
   * - **expressionConfidence**
     - **Not Available**
     - Available

Body Data API
^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :class: longtable

   * - Methods / Data Fields
     - Vulcanexus HRI
     - ROS4HRI
   * - **roi**
     - Available
     - Available
   * - **skeleton**
     - Available
     - Available
   * - **skeleton3d**
     - Available
     - **Not Implemented**
   * - **cropped**
     - **Not Available**
     - Available
   * - **bodyDescription**
     - **Not Available**
     - Available

Person Data API
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :class: longtable

   * - Methods / Data Fields
     - Vulcanexus HRI
     - ROS4HRI
   * - **face**
     - Available
     - Available
   * - **body**
     - Available
     - Available
   * - **voice**
     - Replaced by HRI_STT with `getSpeech()`
     - **Not Available**
   * - **anonymous**
     - Implicit in the keyed topics
     - Available
   * - **engagementStatus**
     - **Not Available**
     - Available
   * - **locationConfidence**
     - **Not Available**
     - Available
   * - **alias**
     - **Not Available**
     - Available
