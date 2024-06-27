#!/bin/bash

# Install base packages
sudo apt install -y lsb-release curl gpg python3-wstool python3-catkin-tools


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


# ----- Clone repositories and install -----
declare -A repositories=(
    ["../rosboard"]="https://github.com/dheera/rosboard.git"
    ["../ethz_piksi_ros"]="https://github.com/ethz-asl/ethz_piksi_ros.git"
    # Add more repositories as needed
)

# Iterate over each repository and check if it already exists
for repo_dir in "${!repositories[@]}"; do
    repo_url=${repositories["$repo_dir"]}
    
    if [ -d "$repo_dir" ]; then
        echo "Repository directory $repo_dir already exists. Skipping cloning."
    else
        git clone "$repo_url" "$repo_dir"
    fi
done


# ----- Install DataSpeed Drive-By-Wire -----
bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_polaris_ros/raw/master/dbw_polaris/scripts/sdk_install.bash)


# ----- Install Piksi Multi GNSS packages -----
cd ..
wstool init
wstool set --git ethz_piksi_ros https://github.com/ethz-asl/ethz_piksi_ros.git
wstool update

./ethz_piksi_ros/piksi_multi_cpp/install/prepare-jenkins-slave.sh

wstool merge ethz_piksi_ros/piksi_multi_cpp/install/dependencies_https.rosinstall
wstool update -j8


# ----- Build workspace -----
catkin build catkin_simple
catkin build libsbp_catkin libsbp_ros_msgs
catkin build libserialport_catkin
catkin build ethz_piksi_ros
catkin build actor_ros rosboard


# ----- Skiplist packages -----
catkin config --skiplist catkin_simple libsbp_catkin libsbp_ros_msgs libserialport_catkin ethz_piksi_ros rosboard