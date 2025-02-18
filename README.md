Ubuntu 22.04 WSL2, ROS2 Humble, YASMIN, and Turtlebot3 Installation Guide
This guide provides step-by-step instructions to install and configure WSL2 on Windows 11, set up Ubuntu 22.04, and install ROS2 Humble along with YASMIN and Turtlebot3. Use this as a reference to quickly set up your development environment.

Table of Contents
Overview
Prerequisites
Installation Steps
1. Enable WSL
2. Set WSL 2 as the Default Version
3. Complete Linux Distribution Setup
4. Install ROS2 Humble
5. Create a ROS2 Workspace
6. Install YASMIN
7. Install Turtlebot3
Additional Resources
Overview
This repository contains instructions to:

Enable the Windows Subsystem for Linux (WSL) on Windows 11.
Install Ubuntu 22.04 as the default WSL distribution.
Set up and update Ubuntu.
Install ROS2 Humble and test it using example nodes.
Create a ROS2 workspace and run Turtlesim.
Install YASMIN from GitHub and run demo applications.
Set up and test Turtlebot3, including keyboard control and custom FSM scripts.
Prerequisites
Operating System: Windows 11
Administrator Access: Required for enabling WSL and installing system updates
Internet Connection: Required to download updates and packages
Installation Steps
1. Enable WSL
Open PowerShell as Administrator:
Right-click the Start button and select Windows Terminal (Admin) or PowerShell (Admin).
Install WSL:
Run the following command to install WSL 2 along with the latest Linux kernel and a default Ubuntu 22.04 distribution:
powershell
Copy
Edit
wsl --install
Restart Your Computer:
After installation, you will be prompted to restart.
Launch Ubuntu 22.04:
Open Ubuntu 22.04 from the Start menu. On the first run, you will be prompted to create a username and password.
2. Set WSL 2 as the Default Version
Open PowerShell as Administrator.
Set WSL 2 as Default:
Execute the following command:
powershell
Copy
Edit
wsl --set-default-version 2
Verify the Version:
Check your Linux distributions and versions by running:
powershell
Copy
Edit
wsl --list --verbose
(Optional) Additional Distributions:
You can install additional distributions from the Microsoft Store if needed.
3. Complete Linux Distribution Setup
User Setup:
When Ubuntu 22.04 is launched for the first time, create your Linux user account.
Update the System:
It is recommended to update your system:
bash
Copy
Edit
sudo apt update && sudo apt upgrade
4. Install ROS2 Humble
Open Ubuntu 22.04 Terminal.
Follow Official ROS2 Instructions:
Refer to the ROS2 Humble Installation Guide to install the required deb packages.
Test Installation:
Run the talker and listener examples to verify that ROS2 is installed correctly.
Add ROS2 to Your Shell Startup:
Append the ROS2 setup script to your .bashrc:
bash
Copy
Edit
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
5. Create a ROS2 Workspace
Workspace Setup:
Follow the Creating a ROS2 Workspace tutorial to set up your workspace.
Test with Turtlesim:
Verify your workspace by running the Turtlesim example as described in the Turtlesim Tutorial.
6. Install YASMIN
Installation:
Follow the instructions provided on the YASMIN GitHub page to install YASMIN.
Run Demos:
Execute the demo applications to confirm that YASMIN is set up correctly.
7. Install Turtlebot3
Installation and Testing:
Follow the ROS2 Turtlebot3 Tutorial to install and test Turtlebot3.
Keyboard Control & Custom FSM:
Use the following commands to set up and control Turtlebot3:
bash
Copy
Edit
# Install Turtlebot3 packages
sudo apt install ros-humble-turtlebot3*

# Set the Turtlebot3 model to 'burger'
export TURTLEBOT3_MODEL=burger

# Update the Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/

# Launch an empty Gazebo world
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Run teleop keyboard control
ros2 run turtlebot3_teleop teleop_keyboard

# Launch Turtlesim node for additional testing
ros2 run turtlesim turtlesim_node

# Run a custom teleop FSM script
python3 complex_teleop_fsm.py

# Launch the YASMIN viewer node
ros2 run yasmin_viewer yasmin_viewer_node

# Make the FSM script executable (if not already)
chmod +x complex_teleop_fsm.py

Additional Resources
ROS2 Humble Installation (Ubuntu Deb Packages)
Creating a ROS2 Workspace
Turtlesim Tutorial
YASMIN GitHub Repository
ROS2 Turtlebot3 Tutorial
