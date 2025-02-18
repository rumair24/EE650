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
- **Administrator Access:** Required for enabling WSL and installing system updates  
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
