<p align="center"><a href="https://vulcanexus.org/" target="_blank" rel="noopener noreferrer"><img src="resources/images/vulcanexus_banner.png"></a></p>

# Vulcanexus

Vulcanexus is a ROS 2 (Robot Operating System) all-in-one tool set. It allows users to build robotics applications combining the unique Vulcanexus elements with the ROS 2 libraries, having Fast DDS as its fixed middleware implementation.

These open source elements include numerous features and tools, providing Vulcanexus users customizable solutions while improving overall system performance. With Vulcanexus, users have fast access to constantly improving functionalities, such as the latest Fast DDS version along with its new features.

Visit the website [vulcanexus.org](https://vulcanexus.org/) and download the package of your choice for free!

## Commercial support

Looking for commercial support? Write us to info@eprosima.com

Find more about us at [eProsima’s webpage](https://eprosima.com/).

## Vulcanexus project

Vulcanexus is composed of a collection of downloadable packages:

*   **Vulcanexus Core**: a set of software libraries that enables users to build the most comprehensive and straightforward robotics application.
    It consists of eProsima Fast DDS latest version as the ROS 2 middleware.
*   **Vulcanexus Tools**: a set of features and applications which allows users to test, improve and configure the performance of Vulcanexus in their systems.
*   **Vulcanexus Micro**: provides access for resource constrained devices (micro-controllers) to the DDS world, bridging the gap between them and ROS 2.
*   **Vulcanexus Cloud**: scales and integrates ROS 2 networks located in geographically spaced environments, and enables the deployment of DDS entities in the cloud in a quick and easy way.
*   **Vulcanexus Simulation**: enables users to design robotic simulations, providing an end-to-end development environment to model, program, and simulate robots.
*   **Vulcanexus Base**: includes within the same installation Vulcanexus tools, micro and cloud features.
*   **Vulcanexus Desktop**: includes ROS 2 desktop installation as well as every available Vulcanexus package.
    It is the more complete Vulcanexus installation.

# Documentation

Vulcanexus official documentation can be found [here](https://docs.vulcanexus.org/en/latest/).

## CI

| Version | Status |
| - | - |
| Rolling | [![docs-rolling](https://github.com/eProsima/vulcanexus/actions/workflows/docs.yaml/badge.svg?branch=main&event=push)](https://github.com/eProsima/vulcanexus/actions/workflows/docs.yaml)
| Humble | [![docs-humble](https://github.com/eProsima/vulcanexus/actions/workflows/docs.yaml/badge.svg?branch=humble&event=push)](https://github.com/eProsima/vulcanexus/actions/workflows/docs.yaml)
| Galactic | [![docs-galactic](https://github.com/eProsima/vulcanexus/actions/workflows/docs.yaml/badge.svg?branch=galactic&event=push)](https://github.com/eProsima/vulcanexus/actions/workflows/docs.yaml)

## Getting started

*   [Installation Manual](https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html): Instructions to set up ROS 2 with Vulcanexus for the first time.
*   [Overview](https://docs.vulcanexus.org/en/latest/rst/introduction/introduction.html): Vulcanexus concepts including ROS 2 and micro-ROS Documentation.
*   [Tutorials](https://docs.vulcanexus.org/en/latest/rst/tutorials/core/ros2_tutorials.html): Collection of step-by-step instructions that help users to get started.
*   [Support](https://docs.vulcanexus.org/en/latest/rst/platforms/platforms.html): Vulcanexus officially supported platforms and releases.
*   [Appendixes](https://docs.vulcanexus.org/en/latest/rst/appendixes/glossary.html): Glossary of terms used throughout this documentation.


## Build documentation (advanced)

Vulcanexus documentation is generated using [Sphinx](https://www.sphinx-doc.org).

### Dependencies

Before being able to build the documentation, some dependencies need to be installed:

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    python3 \
    python3-pip \
    python3-venv \
    python3-sphinxcontrib.spelling \
    imagemagick \
    plantuml
```

### Setup local repository

```bash
# Clone repository
git clone --recursive https://github.com/eProsima/vulcanexus.git
```

#### Virtual environment

This tutorial will create a python3 virtual environment to avoid polluting user's python installation.

```bash
# Create a python3 virtual environment
python3 -m venv vulcanexus_venv
# Activate the environment
source vulcanexus_venv/bin/activate
# Install dependencies within the environment
pip3 install -r vulcanexus/docs/requirements.txt
```

#### Generate HTML docs

```bash
# Source the python virtual environment
source vulcanexus_venv/bin/activate
# Change directories to the repository directory
cd vulcanexus/docs
# Make sure that there are no build directories
make clean
# Generate HTML documentation
make html
# Open the documentation
xdg-open build/html/index.html
```
