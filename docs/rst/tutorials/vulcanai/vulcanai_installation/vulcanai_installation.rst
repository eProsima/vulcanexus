.. _vulcanai_installation:

VulcanAI Installation
=====================

This tutorial provides step-by-step instructions to install and configure *VulcanAI*.
For more info about *VulcanAI* and its capabilities, please refer to the main :ref:`VulcanAI <tutorials_vulcanai_vulcanai_tutorials>` page.

Requirements
------------

- Python 3.10 or higher

Installation via 'pip'
----------------------

The fastest and simplest way to install VulcanAI is using 'pip'.
To do so, open a terminal and run the following command:

.. code-block:: bash

   pip install eprosima-vulcanai

This will install the latest stable version of VulcanAI and all its dependencies.
However, it is recommend to create a virtual environment before installing VulcanAI to avoid potential conflicts with other Python packages.

To create a virtual environment, run the following command:

.. code-block:: bash

   python -m venv vulcanai_venv && \
   source vulcanai_venv/bin/activate

This will create a new virtual environment named `vulcanai_venv` and activate it.
Now, you can proceed to install VulcanAI using the `pip install` command mentioned above.

If venv or pip are not installed in your system, you can install them using your package manager with the following commands:

.. code-block:: bash

   sudo apt update && \
   sudo apt install python3-venv python3-pip

Installation from source
------------------------

If you prefer to install VulcanAI from source, follow these steps:

1. Clone the repository in the desired location and navigate to it:

   .. code-block:: bash

      git clone **TODO: Add repository URL** VulcanAI && cd VulcanAI

2. Install VulcanAI:

   .. code-block:: bash

      python3 -m pip install .

Again, it is recommended to use a virtual environment to avoid potential conflicts with other Python packages.
You can create and activate a virtual environment as explained in the previous section before running the `pip install` command.

Verification
------------

To check the installation, you can run the following command to see if pip is able to list the package:

.. code-block:: bash

   pip show eprosima-vulcanai

.. _vulcanai_from_venv:

Running VulcanAI from virtual environment with Vulcanexus
---------------------------------------------------------

VulcanAI python dependencies might conflict with the ones provided by Vulcanexus.
To avoid this, it is recommended to create a virtual environment for VulcanAI as explained in the :ref:`Installation via 'pip' <vulcanai_installation>` section.
However, virtual environments have their own Python path, so Vulcanexus ROS packages will not be found when running VulcanAI from the virtual environment.
This section explains how to properly fix this to configure the environment to run VulcanAI with Vulcanexus.
It also applies to any other Python virtual environments.

To run VulcanAI with Vulcanexus, you need to have a Vulcanexus distribution installed in your system.
You can follow the official Vulcanexus installation guide for :ref:`binaries <linux_binary_installation>` or :ref:`docker <docker_installation>`.

Once you have a Vulcanexus distribution installed and a virtual environment for VulcanAI created and activated, you need to append the VulcanAI virtual environment's Python path to the `PYTHONPATH` environment variable.
In this way, both Vulcanexus and VulcanAI will be able to find the required Python packages.

To do so, source Vulcanexus and then run the following command in the terminal where you want to run VulcanAI:

.. code-block:: bash

    source /opt/vulcanexus/${VULCANEXUS_DISTRO}/setup.bash && \
    export PYTHONPATH='/<path/to/vulcanai_venv>/lib/python3.x/site-packages':$PYTHONPATH
