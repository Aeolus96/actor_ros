#!/bin/bash

# Install base packages
sudo apt install lsb-release curl gpg


# ----- Install requirements.txt -----
# Check Python version
PYTHON_VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:3])))')

if [[ $(python3 -c "import sys; print(sys.version_info >= (3, 8))") == "False" ]]; then
    echo "Python version 3.8 or higher is required."
    exit 1
fi

# Check if pip3 is installed
if ! command -v pip3 &> /dev/null
then
    echo "pip3 is not installed. Please install pip3 manually."
    exit 1
fi

# Install requirements
pip3 install -r requirements.txt


# ----- Install Redis -----
# Import the Redis GPG key
curl -fsSL https://packages.redis.io/gpg | sudo gpg --dearmor -o /usr/share/keyrings/redis-archive-keyring.gpg
sudo chmod 644 /usr/share/keyrings/redis-archive-keyring.gpg

# Add the Redis repository to the sources list
echo "deb [signed-by=/usr/share/keyrings/redis-archive-keyring.gpg] https://packages.redis.io/deb $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/redis.list

# Update the package index
sudo apt-get update

# Install Redis
sudo apt-get install -y redis-stack-server
sudo apt-get install -y redis


# ----- Make python scripts executable -----
sudo chmod +x scripts/*.py


# ----- Download ROSboard into .../src/ -----

git clone https://github.com/dheera/rosboard.git ../rosboard

catkin build actor_ros rosboard