.. _docker_installation:

Docker installation
===================

*Vulcanexus* offers the possibility of running from a containerized environment by providing a Docker image which contains *Vulcanexus*'s Desktop installation.
This Docker image can be found in `Vulcanexus's Downloads <https://vulcanexus.org/download>`_.
To run it, first install Docker:

.. code-block:: bash

    sudo apt install docker.io

And then load the image with:

.. code-block:: bash

    docker load -i ubuntu-vulcanexus-galactic-desktop.tar

*Vulcanexus* Docker image can be run with:

.. tabs::

    .. tab:: GUI support

        .. code-block:: bash

            xhost local:root
            docker run \
                -it \
                --privileged \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                ubuntu-vulcanexus:galactic-desktop

    .. tab:: CLI support

        .. code-block:: bash

            docker run -it ubuntu-vulcanexus:galactic-desktop

To run more than one session within the same container, *Vulcanexus* installation must be sourced.
Given a running container, you can open another session by:

.. code-block:: bash

    docker exec -it <running-container-id> bash

Then, within the container, source the *Vulcanexus* installation with:

.. code-block:: bash

    source /opt/vulcanexus/galactic/setup.bash

To verify that the sourcing was correct, run:

.. code-block:: bash

    echo $VULCANEXUS_HOME

The output should be:

.. code-block:: bash

    /opt/vulcanexus/galactic

Please refer to :ref:`vulcanexus_tutorials` to read about all the *Vulcanexus* possibilities and getting started with developing for *Vulcanexus*.
