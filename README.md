[![docs-rolling](https://github.com/eProsima/vulcanexus/actions/workflows/docs.yaml/badge.svg?branch=main)](https://github.com/eProsima/vulcanexus/actions/workflows/docs.yaml)
[![docs-galactic](https://github.com/eProsima/vulcanexus/actions/workflows/docs.yaml/badge.svg?branch=galactic)](https://github.com/eProsima/vulcanexus/actions/workflows/docs.yaml)

# Vulcanexus

eProsima extended ROS 2 distribution

## Build documentation

*Vulcanexus Documentation* is generated using [Sphinx](https://www.sphinx-doc.org).

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
