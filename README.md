# Ubuntu 22.04 WSL2, ROS2 Humble, YASMIN, and Turtlebot3 Installation Guide with FSM, BT, and Planning

This guide provides step-by-step instructions for installing and configuring WSL2 on Windows 11, setting up Ubuntu 22.04, and installing ROS2 Humble, YASMIM, and TurtleBot3 with FSM, BT, and Planning. Use it as a reference to quickly set up your development environment.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation Steps](#installation-steps)
  - [1. Enable WSL](#1-enable-wsl)
  - [2. Set WSL 2 as the Default Version](#2-set-wsl-2-as-the-default-version)
  - [3. Complete Linux Distribution Setup](#3-complete-linux-distribution-setup)
  - [4. Install ROS2 Humble](#4-install-ros2-humble)
  - [5. Create a ROS2 Workspace](#5-create-a-ros2-workspace)
  - [6. Install YASMIN](#6-install-yasmin)
  - [7. Install Turtlebot3](#7-install-turtlebot3)
  - [8. Install py_trees](#8-install-py_trees)
  - [9. BT Examples using py_trees](#9-examples-using-py_trees)
  - [10. Integrating ROS2 with py_trees](#10-integrating-ros2-with-py_trees)
  - [11. Simulate TurtleBot with BTs](#11-simulate-turtlebot-with-bts)
  - [12. Install PlanSys2 for Planning](#12-install-plansys2-for-planning)

- [Additional Resources](#additional-resources)

## Overview

This repository contains instructions to:
- Enable the Windows Subsystem for Linux (WSL) on Windows 11.
- Install Ubuntu 22.04 as the default WSL distribution.
- Set up and update Ubuntu.
- Install ROS2 Humble and test it using example nodes.
- Create a ROS2 workspace and run Turtlesim.
- Install YASMIN from GitHub and run demo applications.
- Set up and test Turtlebot3, including keyboard control and custom FSM scripts.
- Install PlanSys2 for planning tasks in ROS2.

## Prerequisites

- **Operating System:** Windows 11  
- **Internet Connection:** Required to download updates and packages

## Installation Steps

### 1. Enable WSL

1. **Open PowerShell as Administrator:**  
   Right-click the Start button and select **Windows Terminal (Admin)** or **PowerShell (Admin)**.
2. **Install WSL:**  
   Run the following command to install WSL 2 along with the latest Linux kernel and a default Ubuntu 22.04 distribution:
   ```powershell
   wsl --install -d Ubuntu-22.04

3. **Restart Your Computer:**  
   After installation, you will be prompted to restart.

4. **Launch Ubuntu 22.04:**  
   Open Ubuntu 22.04 from the Start menu. On the first run, you will be prompted to create a username and password.

### 2. Set WSL 2 as the Default Version

1. **Open PowerShell as Administrator.**
2. **Set WSL 2 as Default:**  
   Execute the following command:
   ```powershell
   wsl --set-default-version 2
3. **Verify the Version:**  
   Check your Linux distributions and versions by running:
   ```powershell
   wsl --list --verbose
4. **(Optional)** You can install any distributions from Microsoft store now. Search distribution, install and launch.
5. To check linux distributions run command:
   ```powershell
   wsl -l
6. To set a default distribution run:
   ```powershell
   wsl --setdefault Ubuntu-22.04
### 3. Complete Linux Distribution Setup

1. **User Setup:**  
   When Ubuntu 22.04 is launched for the first time, create your Linux user account.
2. **Update the System:**  
   It is recommended to update your system:
   ```bash
   sudo apt update && sudo apt upgrade
### 4. Install ROS2 Humble

1. **Open Ubuntu 22.04 Terminal.**
2. **Follow Official ROS2 Instructions:**  
   Refer to the [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install the required deb packages.
3. **Test Installation:**  
   Run the talker and listener examples to verify that ROS2 is installed correctly.
4. **Add ROS2 to Your Shell Startup:**  
   Append the ROS2 setup script to your `.bashrc`:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
### 5. Create a ROS2 Workspace

1. **Workspace Setup:**  
   Follow the [Creating a ROS2 Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) tutorial to set up your workspace.
2. **Test with Turtlesim:**  
   Verify your workspace by running the Turtlesim example as described in the [Turtlesim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).

### 6. Install YASMIN

1. **Installation:**  
   Follow the instructions provided on the [YASMIN GitHub page](https://github.com/uleroboticsgroup/yasmin?tab=readme-ov-file) to install YASMIN.
2. **Run Demos:**  
   Execute the demo applications to confirm that YASMIN is set up correctly.

### 7. Install Turtlebot3

1. **Installation and Testing:**  
   Follow the [ROS2 Turtlebot3 Tutorial](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html) to install and test Turtlebot3.
   ```bash
   # Install Turtlebot3 packages
   sudo apt install ros-humble-turtlebot3*
3. **Keyboard Control & Custom FSM:**  
   Use the following commands to set up and control Turtlebot3:
   ```bash   
   # Set the Turtlebot3 model to 'burger'
   export TURTLEBOT3_MODEL=burger
   
   # Update the Gazebo model path
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/
   
   # Launch an empty Gazebo world
   ros2 launch turtlebot3_gazebo empty_world.launch.py

   #Keyboard Control: Open a new terminal
   # Set the Turtlebot3 model to   'burger'
   export TURTLEBOT3_MODEL=burger
   
   # Run teleop keyboard control
   ros2 run turtlebot3_teleop teleop_keyboard

   # Make the FSM script executable (if not already)
   chmod +x complex_teleop_fsm.py

   # Run a custom teleop FSM script
   python3 complex_teleop_fsm.py
   
   # Launch the YASMIN viewer node
   ros2 run yasmin_viewer yasmin_viewer_node
### 8. Install py_trees
1. py_trees docs: [py_trees documentation](https://py-trees.readthedocs.io/en/devel/index.html)
2. py_trees github: [py_trees github](https://github.com/splintered-reality/py_trees)
3. py_trees_ros docs: [py_trees_ros documentation](https://py-trees-ros.readthedocs.io/en/devel/)
4. py_trees_ros github: [py_trees_ros github](https://github.com/splintered-reality/py_trees_ros)
5. py_trees_ros_tutorials docs: [py_trees_ros_tutorials documentation](https://py-trees-ros-tutorials.readthedocs.io/en/devel/index.html)
6. py_trees_ros_tutorials github: [py_trees_ros_tutorials github](https://github.com/splintered-reality/py_trees_ros_tutorials)
 ```bash
   # Install py_trees & py_trees_ros
   sudo apt update
   sudo apt install ros-humble-py-trees \
                 ros-humble-py-trees-ros-interfaces \
                 ros-humble-py-trees-ros \
                 ros-humble-py-trees-ros-tutorials \
                 ros-humble-py-trees-ros-viewer
```
Test py_trees and py_trees_ros_tutorials:
 ```bash
  # Run py_trees 'action' bahviour demo
    py-trees-demo-action-behaviour
  # Run Tutorial from py_trees_ros_tutorial
    ros2 launch py_trees_ros_tutorials tutorial_one_data_gathering_launch.py
  # View the Behavior Tree with PyTrees Viewer (run in seperate shell)
    py-trees-tree-viewer
```

### 9. BT Examples using py_trees

 ```bash
1. # Navigate to your workspace source directory
cd ros2_wc/src or cd ~/ros2_wc/src 

# Create a new directory for the py_trees_demo package
mkdir py_trees_demo

# Create a Python script for the behavior tree
touch py_trees_demo/first_bt.py

# Make the script executable
chmod +x py_trees_demo/first_bt.py

# Source the environment 
source ~/ros2_ws/install/setup.bash

# Run the Python script from the src folder
python3 py_trees_demo/first_bt.py
```
 ```bash
2. # Navigate to the ROS2 or your workspace source directory
cd ros2_wc/src or cd ~/ros2_wc/src 

# Create a Python script for the selector behavior tree
touch py_trees_demo/selector.py

# Make the script executable
chmod +x py_trees_demo/selector.py

# Source the environment 
source ~/ros2_ws/install/setup.bash

# Run the Python script from the src folder
python3 py_trees_demo/selector.py
```

 ```bash
3. # Navigate to the ROS2 or your workspace source directory
cd ros2_wc/src or cd ~/ros2_wc/src 

# Create a Python script for the sequence behavior tree
touch py_trees_demo/sequence.py

# Make the script executable
chmod +x py_trees_demo/sequence.py

# Source the environment
source ~/ros2_ws/install/setup.bash

# Run the Python script from the src folder
python3 py_trees_demo/sequence.py
```

 ```bash
4. # Navigate to the ROS2 or your workspace source directory
cd ros2_wc/src or cd ~/ros2_wc/src

# Create a Python script for the blackboard behavior tree
touch py_trees_demo/blackboard.py

# Make the script executable
chmod +x py_trees_demo/blackboard.py

# Source the environment
source ~/ros2_ws/install/setup.bash

# Run the Python script from the src folder
python3 py_trees_demo/blackboard.py
```



### 10. Integrating ROS2 with py_trees

In this example, we will implement the following scenario:  
A robot must navigate towards a target. However, it has a battery that needs to be managed. If the battery level drops below a certain threshold, the robot must navigate to a charging station. Here, we are implementing the navigation function, but only the behavior tree (BT) that commands the robot to move towards the target or an imaginary charging station.

#### Hints:  
The robot's battery and its discharge process are emulated using a ROS 2 node, and the battery level is shared using a topic. The behavior tree program reads the battery level from the topic and, based on its level, decides what to do.

---

![BT Navigation Task](BT%20Navigation%20Task.jpg)

### OPTION 1: Git clone Package for the Battery Emulator and Behavior Tree

Git clone the package [Battery Checker Exec]([https://github.com/afaroo01/battery_checker_exec.git].



### OPTION 2: Creating a ROS 2 Package for the Battery Emulator and Behavior Tree

**Step 1: Create a ROS 2 Package** 
```bash
# Create a ROS 2 package with dependencies
ros2 pkg create battery_checker_exec --build-type ament_python --dependencies rclpy sensor_msgs std_msgs py_trees

# Navigate to the package directory
colcon_cd battery_checker_exec
cd battery_checker_exec
```

**Step 2: Create the Python Scripts**
```bash
# Create the battery manager script
touch battery_manager.py

# Make the script executable
chmod +x battery_manager.py

# Create the behavior tree script
touch switching_tasks_tree.py

# Make the script executable
chmod +x switching_tasks_tree.py
```

**Step 3: Modify *setup.py* to Include ROS 2 Nodes**

Before compiling the workspace, modify the setup.py file by adding the following entry points to define the ROS 2 nodes:

```bash
entry_points={
  'console_scripts': [
      'battery_manager = battery_checker_exec.battery_manager:main',
      'switching_tasks = battery_checker_exec.switching_tasks_tree:main',
  ],
},
```

**Step 4: Source the Environment and Run the Nodes**
```bash
# Source the ROS 2 environment
source ~/ros2_ws/install/setup.bash

# Run the behavior tree node
ros2 run battery_checker_exec switching_tasks

# Run the battery manager node
ros2 run battery_checker_exec battery_manager
```

### 11. Simulate TurtleBot with BTs
```bash
# Navigate to your ROS 2 workspace source directory
cd ~/ros2_wc/src

# Clone the repository from GitHub
git clone https://github.com/afaroo01/turtlebot_simulation.git

# Navigate back to the workspace root
cd ~/ros2_wc

# Build the package
colcon build

# Source the environment
source install/setup.bash
```
Install the ROS2 navigation package(the Nav2 package)
```bash
sudo apt-get install ros-humble-nav2-bringup
```
After installing this package, run the simulation using the following command:
```bash
ros2 launch turtlebot_simulation start_simu.launch.py
```
 *Note that the simulation is initially paused. To start it, press the play button located in the bottom-left corner of the user interface.* 

**TO DO TASK** 

To add BT to your launch file, you need to:

1. Create a BT XML file to define behaviors.
2. Write a ROS 2 node to execute the BT.
3. Integrate this node into the launch file.
4. Modify the BT to interact with the Nav2 stack for navigation tasks.

*See the [Turtlebot with BTs.txt](./docs/Turtlebot%20with%20BTs.txt) file for hints*.


### 12. Install and Set Up Plansys

```bash
# Navigate to your ROS 2 workspace source directory
cd ~/ros2_ws/src

# Clone the Plansys repository from GitHub
git clone https://github.com/Plansys/ros2_plansys.git

# Navigate back to the workspace root
cd ~/ros2_ws

# Build the package
colcon build

# Source the environment
source install/setup.bash

# Verify the installation by checking available nodes
ros2 node list

# Run a sample Plansys planner
ros2 run plansys planner_node
```



## Additional Resources

- [ROS2 Humble Installation (Ubuntu Deb Packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Creating a ROS2 Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [Turtlesim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [YASMIN GitHub Repository](https://github.com/uleroboticsgroup/yasmin?tab=readme-ov-file)
- [ROS2 Learning Guide](https://github.com/rumair24/EE650/raw/main/ROS2%20Learning.pdf)


```bash

sudo apt update      # Updates package lists (but does not install updates)
sudo apt upgrade -y  # Installs available updates for installed packages
sudo apt full-upgrade -y  # Upgrades packages, removing obsolete dependencies if needed
sudo apt autoremove -y  # Removes unnecessary packages
sudo apt clean       # Clears local package cache to free up space
```
