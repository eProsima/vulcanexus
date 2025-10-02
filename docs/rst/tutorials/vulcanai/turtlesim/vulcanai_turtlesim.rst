.. _tutorials_vulcanai_turtlesim_tutorials:

VulcanAI with TurtleSim
=======================

:ref:`VulcanAI <tutorials_vulcanai_vulcanai_tutorials>` can be easily adapted to control the popular TurtleSim simulator, which is often used for educational purposes in the ROS 2 community.
The popularity of this simulator makes it an excellent choice for demonstrating the capabilities of VulcanAI in a ROS 2 environment.
This tutorial guides you through the steps to set up and use VulcanAI to control TurtleSim and accomplish complex tasks from natural language instructions, demonstrating its capabilities in a simple yet effective manner.
At the end of this set of tutorials you will have learned:

- How to use most of VulcanAI's features.
- How to create custom VulcanAI tools, adapted to a ROS 2 environment using TurtleSim as an example.
- How to use VulcanAI to control TurtleSim through the VulcanAI console.

At the end of this set of tutorials, you will be able to achieve the following behaviors found in the next video:

.. raw:: html

      <div class="video-center">
        <video controls playsinline muted loop>
          <source src="../../../../_static/resources/tutorials/vulcanai/pentagon.mp4">
        </video>
      </div>

Requisites
----------

Before starting this tutorial, make sure you have a working installation of Vulcanexus and VulcanAI.
If you haven't installed them yet, please refer to the :ref:`Vulcanexus Installation <linux_binary_installation>` guide and :ref:`VulcanAI Installation <vulcanai_installation>` tutorial for step-by-step instructions on how to set up VulcanAI. Then, install the TurtleSim package:

.. code-block:: bash

    sudo apt update && \
    sudo apt install ros-kilted-turtlesim

Although not mandatory, it is recommended to go through the :ref:`VulcanAI Tools <vulcanai_tools>` tutorial to familiarize yourself with the concept of *tools* in VulcanAI, as they are essential for creating custom functionalities and extending VulcanAI's capabilities, tailoring them to control TurtleSim.

As VulcanAI relies on Large Language Models (LLMs) to understand and execute user instructions, it is needed to have access to an LLM service.
VulcanAI supports multiple LLM providers such as OpenAI or Google GenAI.
However, this tutorial has been developed and tested using OpenAI's models, so it is recommended to use this provider to avoid potential misbehaviors.
In any way, an API key from the chosen provider is needed to use VulcanAI.

Next steps
----------

Start the tutorial by following the next steps:

.. toctree::
    :maxdepth: 1

    turtlesim_tools/turtlesim_tools.rst
    turtlesim_main/turtlesim_main.rst
    turtlesim_usage/turtlesim_usage.rst
