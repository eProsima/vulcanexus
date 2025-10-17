.. _vulcanai_introduction:

VulcanAI Overview
=================

VulcanAI is a library that provides a framework to easily and flexibly create powerful AI applications.
Leveraging LLM capabilities, VulcanAI acts as an intelligent assistant, capable of understanding and
executing complex tasks by decomposing them into manageable sub-tasks in any environment.
VulcanAI excels at planning and reasoning, providing a deeper level of human-context understanding to robotic systems.
VulcanAI is completely compatible with Vulcanexus, and works marvelously to control and even debug ROS 2
robots and applications.

It relies on the concept of *tools* to extend its capabilities and adapt to different environments.
Tools are special components that provide specific functionalities to VulcanAI, allowing its agents to interact
with the external world.
The following sections describe the main features of VulcanAI and introduces its core concepts to start integrating VulcanAI into your own applications.

.. toctree::
    :maxdepth: 1

    vulcanai_installation/vulcanai_installation.rst
    vulcanai_tools/vulcanai_tools.rst
