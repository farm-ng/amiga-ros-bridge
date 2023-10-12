#!/bin/bash

# Get the current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to the current directory
cd $DIR

# Create the virtual environment
apt update && apt install python3.8-venv -y
python3 -m venv $DIR/venv

# Source the virtual environment
source $DIR/venv/bin/activate

# Install the requirements
# pip install -r $DIR/requirements.txt

# Upgrade some dependencies
pip install --upgrade pip
pip install --upgrade setuptools wheel

# Install requirements
pip install -r requirements.txt

# Install the submodule packages
pip install -e farm-ng-core
pip install --no-build-isolation -e farm-ng-amiga
