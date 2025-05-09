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

    docker load -i ubuntu-vulcanexus-jazzy-desktop.tar

*Vulcanexus* Docker image can be run with:

.. tab-set::

    .. tab-item:: GUI support

        .. code-block:: bash

            xhost local:root
            docker run \
                -it \
                --privileged \
                --net host \
                --ipc host \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                ubuntu-vulcanexus:jazzy-desktop

    .. tab-item:: CLI support

        .. code-block:: bash

            docker run -it ubuntu-vulcanexus:jazzy-desktop

To run more than one session within the same container, *Vulcanexus* installation must be sourced.
Given a running container, you can open another session by:

.. code-block:: bash

    docker exec -it <running-container-id> bash

Then, within the container, source the *Vulcanexus* installation with:

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash

To verify that the sourcing was correct, run:

.. code-block:: bash

    echo $VULCANEXUS_HOME

The output should be:

.. code-block:: bash

    /opt/vulcanexus/jazzy
