.. _tutorials_vulcanai_turtlesim_requirements:

Requisites
==========

.. note::
   VulcanAI is currently in active development, and new features and improvements are being added regularly.
   Current version is in Beta stage.

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
