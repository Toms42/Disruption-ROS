

# RCP-Spotlight

This is the repo for the Spotlight project for Robotics for Creative Practice (16-375) at CMU.

## Setup and File Hierarchy
### This stuff all relies on ROS, so install from here:

http://wiki.ros.org/melodic/Installation/Ubuntu

### Next, make sure to install catkin_tools:
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
(If you are not on ubuntu, change 'lsb_release -sc' to 'bionic' without quotes)
	
	wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
	sudo apt-get update
	sudo apt-get install python-catkin-tools
	

### These ROS packages should live in your `catkin_ws` folder. To install:

    cd ~ # or wherever else you want software to live
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone --recurse-submodules https://github.com/Toms42/rcp_spotlight
    rosdep update
    sudo apt-get update
    rosdep install --from-paths rcp_spotlight --ignore-src -r -y

There's a few dependencies I never got around to setting up rosdep stuff for, so install those manually:

    sudo -H pip install smc scipy

To update dependencies (eg, after pulling in a large change) run this command again:

    rosdep update
    sudo apt-get update
    rosdep install --from-paths path/to/rcp_spotlight --ignore-src -r -y
If you are using linux mint (or another unsupported OS), you will need to add the following line to your bashrc and source it again, otherwise commands like rosdep won't work. If your distro doesn't use 18.04 as the upstream, you will need to change this to the right upstream os version.

    export ROS_OS_OVERRIDE=ubuntu:18.04:bionic

### Add this to your bashrc (or zshrc), or run it every boot:

    source /opt/ros/melodic/setup.bash
Or (if you use zsh):

    source /opt/ros/melodic/setup.zsh

### To build your ROS project:

    cd path/to/catkin_ws
    catkin build
    source devel/setup.bash #(or setup.zsh)
