Ubuntu 22.04 WSL2, ROS2 Humble, YASMIN, and Turtlebot3 Installation Guide
# Ubuntu 22.04 WSL2, ROS2 Humble, YASMIN, and Turtlebot3 Installation Guide

This guide provides step-by-step instructions to install and configure WSL2 on Windows 11, set up Ubuntu 22.04, and install ROS2 Humble along with YASMIN and Turtlebot3. Use this as a reference to quickly set up your development environment.

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

## Additional Resources

- [ROS2 Humble Installation (Ubuntu Deb Packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Creating a ROS2 Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [Turtlesim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [YASMIN GitHub Repository](https://github.com/uleroboticsgroup/yasmin?tab=readme-ov-file)
- [ROS2 Turtlebot3 Tutorial](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)
