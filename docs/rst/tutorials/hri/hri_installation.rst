.. _tutorials_hri_installation:

Vulcanexus HRI Installation Tutorial
====================================

Quickview
---------

This tutorial will explain how to install all Vulcanexus HRI dependencies and packages.
In case docker is being used, it will also cover how to create a docker image ready to run every HRI tutorial included in Vulcanexus.

Prerequisites
-------------

Before starting this tutorial, ensure you have the following prerequisites:

- A working installation of Vulcanexus (:ref:`binary <linux_binary_installation>`, :ref:`source <linux_source_installation>` or :ref:`docker <docker_installation>`).

Installation Steps
------------------

Vulcanexus provides a script to facilitate the installation of all HRI-related dependencies and packages.
To install it, just run the following command in your terminal:

.. code-block:: bash

   sudo install_hri

This script will automatically download and install all necessary packages and dependencies required for HRI functionalities in Vulcanexus.
It will also add to the ``bashrc`` any paths or environment variables needed for proper operation.

Note that this script installs apt packages and python packages via pip.
In case a virtual environment is desired, make sure to use the arg ``--venv`` pointing to the root directory of your virtual environment.

.. code-block:: bash

   sudo install_hri --venv /path/to/your/venv

Docker Image Creation
---------------------

In case you are using Docker, you can create a Docker image with all HRI dependencies and packages pre-installed.
To do so, run a Docker container from the Vulcanexus base image and execute the installation script inside the container.

.. code-block:: bash

   docker run -it --name vulcanexus_hri_container eprosima/vulcanexus:kilted-base bash

   # Inside the container terminal
   install_hri

Afterwards, open a new terminal in your host machine and commit the changes made in the container to create a new Docker image.

.. code-block:: bash

   docker commit vulcanexus_hri_container ubuntu-vulcanexus:<VULCANEXUS_DISTRO>-hri

All tutorials in this section uses docker compose files that rely on this image.
Even if your Vulcanexus installation is native, it is recommended to create this image to run the tutorials in Docker containers.
