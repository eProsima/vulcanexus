.. _webots:

Webots
======

*Webots* is an open-source three-dimensional mobile robot simulator.
It was originally developed as a research tool to investigate various control algorithms in mobile robotics. Since December 2018, Webots is released as an open source software under the `Apache 2.0 license <https://www.apache.org/licenses/LICENSE-2.0>`_.

Webots offers a rapid prototyping environment, that allows the user to create 3D virtual worlds with physics properties such as mass, joints, friction coefficients, etc. The user can add simple passive objects or active objects called mobile robots. These robots can have different locomotion schemes (wheeled robots, legged robots, or flying robots). Moreover, they may be equipped with a number of sensor and actuator devices, such as distance sensors, drive wheels, cameras, motors, touch sensors, emitters, receivers, etc. Finally, the user can program each robot individually to exhibit the desired behavior. Webots contains a large number of robot models and controller program examples to help users get started.
Webots also contains a number of interfaces to real mobile robots, so that once your simulated robot behaves as expected, you can transfer its control program to a real robot like e-puck, DARwIn-OP, Nao, etc. Adding new interfaces is possible through the related system.

What can I do with Webots?
--------------------------

Webots is well suited for research and educational projects related to mobile robotics. Many mobile robotics projects have relied on Webots for years in the following areas:

* Mobile robot prototyping (academic research, the automotive industry, aeronautics, the vacuum cleaner industry, the toy industry, hobbyists, etc.).
* Robot locomotion research (legged, humanoids, quadrupeds robots, etc.).
* Multi-agent research (swarm intelligence, collaborative mobile robots groups, etc.).
* Adaptive behavior research (genetic algorithm, neural networks, AI, etc.).
* Teaching robotics (robotics lectures, C/C++/Java/Python programming lectures, etc.).
* Robot contests (e.g. Robotstadium or Rat's Life).


What Do I Need to Know to Use Webots?
-------------------------------------
You will need a minimal amount of technical knowledge to develop your own simulations:

* A basic knowledge of the C, C++, Java, Python or MATLAB programming language is necessary to program your own robot controllers. However, even if you don't know these languages, you can still program the e-puck and Hemisson robots using a simple graphical programming language called BotStudio.
* If you don't want to use existing robot models provided within Webots and would like to create your own robot models, or add special objects in the simulated environments, you will need a basic knowledge of 3D computer graphics and VRML97 description language. That will allow you to create 3D models in Webots or import them from 3D modeling software.

Webots Simulation
-----------------

A Webots simulation is composed of following items:

    1. A Webots world file (.wbt) that defines one or several robots and their environment. The .wbt file does sometimes depend on external PROTO files (.proto) and textures.
    2. One or several controller programs for the above robots (in C/C++/Java/Python/MATLAB).
    3. An optional physics plugin that can be used to modify Webots regular physics behavior (in C/C++).

.. figure:: /rst/figures/intro/tools/webots/webots_nao.png

Next Steps
----------

Visit `Webots Documentation <https://cyberbotics.com/doc/guide/getting-started-with-webots>`_ for more information.
