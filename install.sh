#!/bin/bash

NOCOLOR='\033[0m'
LOGCOLOR='\033[1;36m'
PASSCOLOR='\033[1;32m'
WARNCOLOR='\033[1;31m'

LOGPREFIX="${LOGCOLOR}INSTALLER:"
WARNPREFIX="${WARNCOLOR}ERROR:"
PASSTEXT="${PASSCOLOR}PASS"
FAILTEXT="${WARNCOLOR}FAIL"

instlog() {
    printf "$LOGPREFIX $@ $NOCOLOR\n"
}

instwarn() {
    printf "$WARNPREFIX $@ $NOCOLOR\n"
}


instpass() {
    printf "$PASSTEXT $NOCOLOR"
}


instfail() {
    printf "$FAILTEXT $NOCOLOR"
}

check_host() {

    # Attempts to ping a host to make sure it is reachable
    HOST="$1"

    HOST_PING=$(ping -c 2 $HOST 2>&1 | grep "% packet" | cut -d" " -f 6 | tr -d "%")
    if ! [ -z "${HOST_PING}" ]; then

        # Uses packet loss percentage to determine if the connection is strong
        if [ $HOST_PING -lt 25 ]; then

            # Will return true if ping was successful and packet loss was below 25%
            return `true`
        fi
    fi
    return `false`
}

REQUIRED_OS="xenial"
CATKIN_DIR=~/mil_ws

# The paths to the aliases configuration files
BASHRC_FILE=~/.bashrc
ALIASES_FILE=~/.mil_aliases

#==================#
# Pre-Flight Check #
#==================#

instlog "Starting the pre-flight system check to ensure installation was done properly"

# The lsb-release package is critical to check the OS version
# It may not be on bare-bones systems, so it is installed here if necessary
sudo apt-get update -qq
sudo apt-get install -qq lsb-release

# Ensure that the correct OS is installed
DTETCTED_OS="`lsb_release -sc`"
if [ $DTETCTED_OS = $REQUIRED_OS ]; then
    OS_CHECK=true
    echo -n "[ " && instpass && echo -n "] "
else
    OS_CHECK=false
    echo -n "[ " && instfail && echo -n "] "
fi
echo "OS distribution and version check"


# Prevent the script from being run as root
if [ $USER != "root" ]; then
    ROOT_CHECK=true
    echo -n "[ " && instpass && echo -n "] "
else
    ROOT_CHECK=false
    echo -n "[ " && instfail && echo -n "] "
fi
echo "Running user check"

# Check whether or not github.com is reachable
# This also makes sure that the user is connected to the internet
if (check_host "github.com"); then
    NET_CHECK=true
    echo -n "[ " && instpass && echo -n "] "
else
    NET_CHECK=false
    echo -n "[ " && instfail && echo -n "] "
fi
echo "Internet connectivity check"

if !($OS_CHECK); then

    # The script will not allow the user to install on an unsupported OS
    instwarn "Terminating installation due to incorrect OS (detected $DTETCTED_OS)"
    instwarn "This project requires Ubuntu Xenial (16.04)"
    exit 1
fi

if !($ROOT_CHECK); then

    # The script will not allow the user to install as root
    instwarn "Terminating installation due to forbidden user"
    instwarn "The install script should not be run as root"
    exit 1
fi

if !($NET_CHECK); then

    # The script will not allow the user to install without internet
    instwarn "Terminating installation due to the lack of an internet connection"
    instwarn "The install script needs to be able to connect to GitHub and other sites"
    exit 1
fi

#===================================================#
# Repository and Set Up and Main Stack Installation #
#===================================================#

# Make sure script dependencies are installed on bare bones installations
instlog "Installing install script dependencies"
sudo apt-get install -qq wget curl aptitude fakeroot ssh git

#Add software repositories for ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Add software repository for Git-LFS
instlog "Adding the Git-LFS packagecloud repository to software sources"
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash

instlog "Installing Catkin and Rosdep"
sudo apt-get update -qq
sudo apt-get install -qq catkin python-rosdep

instlog "Installing ROS Kinetic base packages"
sudo apt-get install -qq ros-kinetic-desktop-full 

# Source ROS configurations for bash on this user account
source /opt/ros/kinetic/setup.bash
if !(cat $BASHRC_FILE | grep --quiet "source /opt/ros"); then
    echo "" >> $BASHRC_FILE
    echo "# Sets up the shell environment for ROS" >> $BASHRC_FILE
    echo "source /opt/ros/kinetic/setup.bash" >> $BASHRC_FILE
fi

# Get information about ROS versions
instlog "Initializing ROS"
if !([ -f /etc/ros/rosdep/sources.list.d/20-default.list ]); then
    sudo rosdep init > /dev/null 2>&1
fi
rosdep update

instalog "Installing Sub8 ROS dependencies"
sudo apt-get install -qq binutils-dev
sudo apt-get install -qq ros-kinetic-ompl
sudo apt-get install -qq ros-kinetic-spacenav-node

# Get older version of Google protobuf for Gazebo
instlog "Installing Google protobuf v2.6.1"
wget https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz
tar -xvzf protobuf-2.6.1.tar.gz
cd protobuf-2.6.1
./configure
make
make check
sudo make install
cd ..
sudo rm -r protobuf-2.6.1
sudo rm -r protobuf-2.6.1.tar.gz

# Set up catkin workspace directory
if !([ -f $CATKIN_DIR/src/CMakeLists.txt ]); then
    instlog "Generating catkin workspace at $CATKIN_DIR"
    mkdir -p $CATKIN_DIR/src
    cd $CATKIN_DIR/src
    catkin_init_workspace
else
    instlog "Using existing catkin workspace at $CATKIN_DIR"
    cd $CATKIN_DIR/src
fi

CMAKE_PREFIX_PATH=/opt/ros/kinetic
git clone https://github.com/ros-drivers/driver_common
catkin_make -C $CATKIN_DIR -B

source ../devel/setup.bash

if !(ls $CATKIN_DIR/src | grep --quiet "Sub8"); then
    instlog "Downloading the Sub8 repository"
    cd $CATKIN_DIR/src
    git clone https://github.com/uf-mil/sub8.git
    cd $CATKIN_DIR/src/Sub8
    git remote rename origin upstream
    instlog "Make sure you change your git origin to point to your own fork! (git remote add origin your_forks_url)"
fi

#================================#
#    Dependency Installation     #
#================================#
instlog "Installing Sub8 dependencies"

sudo apt-get install python-scipy
sudo apt-get install tesseract-ocr
sudo apt-get install libusb-1.0-0-dev

# Tools
sudo apt-get install -qq sshfs
sudo apt-get install -qq git-lfs gitk
git lfs install --skip-smudge
sudo apt-get install -qq tmux

# Libraries needed by txros
sudo apt-get install -qq python-twisted socat

# Package management
sudo pip install -q -U setuptools

# Service identity verification
sudo pip install -q -U service_identity

# Utilities
sudo pip install -q -U argcomplete
sudo pip install -q -U tqdm
sudo pip install -q -U pyasn1
sudo pip install -q -U characteristic
sudo pip install -q -U progressbar

# Machine Learning
sudo pip install -q -U scikit-learn > /dev/null 2>&1

# Visualization
sudo pip install -q -U mayavi > /dev/null 2>&1

instlog "Cloning common GIT repositories"
cd $CATKIN_DIR/src
git clone https://github.com/RustyBamboo/camera1394.git
git clone https://github.com/txros/txros.git

mv $CATKIN_DIR/src/sub8/gnc/sub8_trajectory_generator $CATKIN_DIR/src/sub8/gnc/.sub8_trajectory_generator

catkin_make -C $CATKIN_DIR -B
