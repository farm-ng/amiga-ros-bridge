#!/bin/bash

# Get the current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to the current directory
cd $DIR

# Ensure submodules are up to date
git submodule update --init --recursive

# Create the virtual environment
python3 -m venv $DIR/venv

# Source the virtual environment
source $DIR/venv/bin/activate

# Install the requirements
# pip install -r $DIR/requirements.txt

# Upgrade some dependencies
pip install --upgrade pip
pip install --upgrade setuptools wheel

# Install the submodule packages
pip install -e farm-ng-core
pip install --no-build-isolation -e farm-ng-amiga