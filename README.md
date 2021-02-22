# Gazebo3DPrintingSimulator

To use this simulation tool (Tested on Ubuntu version 18.04 and 20.04) follow these instructions :

### Table of Contents

* [Required Dependencies](https://github.com/ai4ce/Gazebo3DPrintingSimulator#install-the-required-dependencies)

* [Build Essentials](https://github.com/ai4ce/Gazebo3DPrintingSimulator#install-build-essentials)

* [Install Gazebo](https://github.com/ai4ce/Gazebo3DPrintingSimulator#install-gazebo)

* [Install the Material Simulation Plugin for Gazebo](https://github.com/ai4ce/Gazebo3DPrintingSimulator#install-gazebo-fluid-simulation-plugin-with-splishsplash)

* [Meshing Tool](https://github.com/ai4ce/Gazebo3DPrintingSimulator#fluid-engine-dev-for-meshing-simulations)

* [Connect to ROS](https://github.com/ai4ce/Gazebo3DPrintingSimulator#ros-to-connect-the-plugin-for-multi-robot-simulations)

  - [Control the Robots](https://github.com/ai4ce/Gazebo3DPrintingSimulator#running-turtlebots)

-----------
## Install the Required dependencies

From [The Gazebo Installation Guide](http://gazebosim.org/tutorials?tut=install_from_source&cat=install)

1. **Setup your computer to accept software from packages.osrfoundation.org**
    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ```

2. **Setup keys and Update:**   

    ```
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    ```   
3. **Install prerequisites. A clean Ubuntu system will need the following (replace version with the major version of gazebo you intend to build, eg: 7, 8, 9. And if using ROS, replace dummy with your ROS version, eg: indigo, jade, kinetic...)**:    
    ```
    wget https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
    GAZEBO_MAJOR_VERSION=version ROS_DISTRO=dummy . /tmp/dependencies.sh
    echo $BASE_DEPENDENCIES $GAZEBO_BASE_DEPENDENCIES | tr -d '\\' | xargs sudo apt-get -y install
    ```
    
   We recommend using ``ROS_DISTRO=meolodic`` and ``GAZEBO_MAJOR_VERSION=9``
   
   
Refer to [The Gazebo Installation from Source](http://gazebosim.org/tutorials?tut=install_from_source&cat=install) for **optional** additional physics engines (not required for the basic functionalities of our tool)

## Install build essentials

```
sudo apt update
sudo apt install build-essentials 
```

## Install Gazebo 
    
1. **clone the repository**
  ```
  git clone https://bitbucket.org/hbpneurorobotics/gazebo.git
  ```
  Note that more recent version of Gazebo may not be fully compatible, which is why we are providing this version
  
2. **Install using the following commands**
  ```
  cd gazebo
  mkdir build
  cd build
  cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local ..
  make -j8
  make install
  ```
  You can use `make -j4` or another lower number than 8 based on the available memory
  
  ## Install Gazebo Fluid Simulation Plugin with SPlisHSPlasH
  
1. **open your bash** `nano ~/.bashrc`

    1. Add a line to the bottom of the file 
        ```
        export LD_LIBRARY_PATH=/home/jason2/.local/lib:$LD_LIBRARY_PATH:/home/<your_User_Name>/splisplash/build/lib
        ```
    2. save changes and exit before runnig: `source ~/.bashrc`
    
2. **Clone our repo**
  ```
  Lorem Ipsum
  ```
  Sit DOlorem.
  
  ## Fluid Engine Dev for meshing simulations
  
1. **Install fluid-engine_dev plugin**

    1. Clone the [Fluid Engine Dev - Jet ](https://github.com/doyubkim/fluid-engine-dev) repo
    2. Build it on your machine accordingly as you did with Gazebo `mkdir build && cd build && cmake .. && make`
    
2. **Locate the particles2obj executable**
    
    e.g `/home/<User_name>/fluid-engine-dev/build/bin/particles2obj`
    1. How to Run the conversion for meshing in general
        ```
        code
        ```
    2. Run it in the command line during run time
        ```
        instructions
        ```
  ## ROS to connect the plugin for multi robot simulations
  
  1. **Desktop Full Install ROS**
        
        1. Follow the [ROS Installation Instructions](http://wiki.ros.org/melodic/Installation/Ubuntu). 
        At **1.4** choose **Desktop-Full Install**
    
2. **Connect ROS to Gazebo**
    
    1. Install [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros#Installgazebo_ros_pkgs)
        ```
        git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b melodic-devel
        ```
    2. Use Rosdep to check for missing pckgs (dont install gazebo9 & libgazebo9-dev)
        ```
        rosdep check --from-paths . --ignore-src --rosdistro melodic
        ```
3. **Install moveit**

        git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b melodic-devel
        
4. **Create a [Catkin Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)**
5. **Download [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make)**

### Running TurtleBots

1. **Create a [Catkin Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)**
2. `cd src`and **[Download turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)**
3.  **Download [ttbot with open-manipulator](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation)** (change to melodic)

