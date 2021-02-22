# Gazebo3DPrintingSimulator

To use this simulation tool (Tested on Ubuntu version 18.04 and 20.04) :

## Install the Required dependencies

From [The Gazebo Installation Guide](http://gazebosim.org/tutorials?tut=install_from_source&cat=install)

1. Setup your computer to accept software from packages.osrfoundation.org    
    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ```

2. Setup keys and Update:    

    ```
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    ```   
3. Install prerequisites. A clean Ubuntu system will need the following (replace version with the major version of gazebo you intend to build, eg: 7, 8, 9. And if using ROS, replace dummy with your ROS version, eg: indigo, jade, kinetic...):    
    ```
    wget https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
    GAZEBO_MAJOR_VERSION=version ROS_DISTRO=dummy . /tmp/dependencies.sh
    echo $BASE_DEPENDENCIES $GAZEBO_BASE_DEPENDENCIES | tr -d '\\' | xargs sudo apt-get -y install
    ```
    
   We recommend using ``ROS_DISTRO=meolodic`` and ``GAZEBO_MAJOR_VERSION=9``
   
   
Refer to [The Gazebo Installation from Source](http://gazebosim.org/tutorials?tut=install_from_source&cat=install) for **optional** additional physics engines (not required for the basic functionalities of our tool)

