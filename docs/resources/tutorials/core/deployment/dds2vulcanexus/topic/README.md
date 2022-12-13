# Fast DDS - Vulcanexus Topic Intercommunication Demo

This project contains code for demonstrating intercommunication between Vulcanexus and Fast DDS applications.
It provides a Vulcanexus application which instantiates a publisher on a `HelloWorld` topic, and a Fast DDS subscriber application that subscribes to it.
Furthermore, it provides a Dockerfile to ease the building and showcasing process.

## Build the Docker image

To build the Docker image, run (from this directory):

```bash
docker build -f Dockerfile -t dds2vulcanexus .
```

## Build the demo

This demo requires two shells, one for the Vulcanexus publisher and another one for the Fast DDS subscriber.

### Run the Fast DDS subscriber

1. Open a shell and run:

    ```bash
    docker run -it --rm dds2vulcanexus
    ```

1. From within the container, run:

    ```bash
    fastdds_subscriber
    ```

The subscriber application will start showing messages as soon as the publisher is run.
It can be killed with `CTRL-C`.

### Run the Fast DDS subscriber

1. Open a shell and run:

    ```bash
    docker run -it --rm dds2vulcanexus
    ```

1. From within the container, run:

    ```bash
    vulcanexus_publisher
    ```

The publisher application will start sending messages that will be received by the Fast DDS subscriber in the other shell.
The publisher can be killed with `CTRL-C` as well.
