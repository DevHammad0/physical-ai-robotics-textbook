---
title: "Setting Up Your ROS 2 Environment"
chapter: 1
lesson: 2
duration_minutes: 45

# Robotics-Specific Metadata
simulation_required: ["turtlesim"]
safety_level: "simulation_only"
cefr_level: "A2"
hardware_prerequisites: []

# Learning Objectives (Measured, SMART)
learning_objectives:
  - "Install ROS 2 Humble on Ubuntu 22.04"
  - "Verify ROS 2 installation with CLI tools"
  - "Launch Turtlesim simulator and control a virtual robot"
  - "Set up Python development environment for ROS 2"
  - "Understand environment variables (ROS_DOMAIN_ID, ROS_DISTRO)"

# Pedagogical Layer
primary_layer: "Layer 1 (Manual Foundation)"
---

## Introduction

Before you can write ROS 2 code, you need a working development environment. This lesson walks you through installing ROS 2 Humble—the current long-term support (LTS) version—and launching Turtlesim, your first robot simulator.

**Time estimate**: 45 minutes (assuming stable internet and no errors)

**Prerequisites**:
- Computer with Ubuntu 22.04 LTS or Windows/macOS with VM/WSL2
- 3GB free disk space
- Internet connection
- Terminal familiarity (cd, ls, sudo commands)

---

## What We'll Do in This Lesson

By the end of this lesson, you'll have:

1. ✓ ROS 2 Humble installed and verified
2. ✓ Turtlesim running in a simulation window
3. ✓ Environment variables properly configured
4. ✓ Python 3.10+ ready for ROS 2 development
5. ✓ Confidence to start writing nodes in Lesson 3

---

## Concept 1: Linux Prerequisites and Environment

ROS 2 officially targets **Ubuntu 22.04 LTS** (Jammy Jellyfish). While ROS 2 runs on other Linux distributions, Ubuntu is the best-supported platform.

### If You Don't Have Ubuntu 22.04

**Option 1: Native Ubuntu**
- Download Ubuntu 22.04 LTS from https://ubuntu.com/download/desktop
- Install on a spare computer or replace your OS

**Option 2: Virtual Machine (Windows/macOS)**
- Download VirtualBox (free): https://www.virtualbox.org/
- Create a VM with Ubuntu 22.04
- Allocate at least 2 CPU cores and 4GB RAM
- Tutorial: https://www.virtualbox.org/wiki/Documentation

**Option 3: WSL2 (Windows Only)**
- Windows Subsystem for Linux 2 lets you run Ubuntu inside Windows
- Setup: https://docs.microsoft.com/en-us/windows/wsl/install
- After installing, run: `wsl --install -d Ubuntu-22.04`

**Option 4: Docker Container (Advanced)**
- Pre-built ROS 2 Docker images are available
- Recommended for experienced developers
- Documentation: https://hub.docker.com/r/osrf/ros

---

## Step 1: Update Your System

Open a terminal and update Ubuntu's package manager:

```bash
sudo apt update
sudo apt upgrade -y
```

This downloads the latest package lists and upgrades existing software. You may be prompted for your password (this is normal).

**Expected output**:
```
Reading package lists... Done
Building dependency tree... Done
The following packages will be upgraded:
  curl git vim [... more packages ...]
Processing triggers for [... more output ...]
```

---

## Step 2: Add the ROS 2 Repository

ROS 2 is not in Ubuntu's default repositories. Add the official ROS 2 repository:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt update
```

**What this does**:
- Line 1: Downloads and adds the ROS 2 GPG key (verifies package authenticity)
- Line 2: Adds the ROS 2 repository to your package sources
- Line 3: Refreshes the package list

**Expected output**:
```
OK
Hit:1 http://archive.ubuntu.com/ubuntu jammy InRelease
Hit:2 http://packages.ros.org/ros2/ubuntu jammy InRelease
```

---

## Step 3: Install ROS 2 Humble

Now install the full ROS 2 Humble desktop distribution:

```bash
sudo apt install -y ros-humble-desktop
```

This installs ROS 2 core, development tools, and visualization utilities. Installation takes 5-10 minutes depending on internet speed.

**What's included**:
- `rclpy`: Python ROS 2 client library
- `rclcpp`: C++ ROS 2 client library
- `ros2`: Command-line tools (ros2 run, ros2 topic, etc.)
- `rviz2`: 3D visualization tool
- `gazebo`: Physics simulator (for later chapters)
- Turtlesim (your first robot simulator)

**Expected output**:
```
Setting up ros-humble-base [version] ...
Setting up ros-humble-desktop [version] ...
Processing triggers for man-db ...
```

**Verification**: After installation, check that ROS 2 is installed:

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

**Expected output**:
```
ROS 2 Humble Hawksbill (version 12.x.x)
```

---

## Step 4: Set Up Environment Sourcing

Every time you open a new terminal, ROS 2 commands won't work until you source the setup file. To automate this, add the sourcing command to your shell startup file.

### Option A: Using Bash (Most Common)

Add the sourcing command to `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Option B: Using Zsh (macOS Default)

Add the sourcing command to `.zshrc`:

```bash
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

**What this does**: Every new terminal will automatically source ROS 2 setup, so you don't need to type it manually.

**Verification**: Close your terminal and open a new one. Then run:

```bash
echo $ROS_DISTRO
```

**Expected output**:
```
humble
```

If you see `humble`, sourcing is working correctly!

---

## Concept 2: ROS Environment Variables

ROS 2 uses environment variables to configure runtime behavior. The three most important are:

### ROS_DISTRO
```bash
echo $ROS_DISTRO
# Output: humble
```
**Meaning**: You're using ROS 2 Humble (not Rolling, Iron, or older LTS versions)

### ROS_DOMAIN_ID
```bash
echo $ROS_DOMAIN_ID
# Output: (empty by default)
```
**Meaning**: Default domain ID is 0. This isolates your ROS 2 network. If you run multiple ROS 2 systems on the same network, give each a different domain ID (1, 2, 3, etc.) to prevent interference.

### ROS_MIDDLEWARE_IMPL
```bash
echo $ROS_MIDDLEWARE_IMPL
# Output: rmw_cyclonedds_cpp (or cyclonedds, connextdds, etc.)
```
**Meaning**: ROS 2 uses DDS (Data Distribution Service) as its middleware. CycloneDDS is the default and most reliable for our purposes.

---

## Step 5: Install Python Development Tools

ROS 2 development requires Python 3.10+, pip, and build tools. Install them:

```bash
sudo apt install -y python3-pip python3-colcon-common-extensions build-essential
```

**What this includes**:
- `python3-pip`: Python package manager
- `colcon`: Build system for ROS 2 packages
- `build-essential`: Compiler toolchain (gcc, g++, make)

**Verification**: Check Python version:

```bash
python3 --version
# Expected: Python 3.10.x or higher
```

---

## Step 6: Launch Turtlesim (Your First Simulation)

Now for the exciting part—launch Turtlesim!

### Terminal 1: Launch the Turtlesim Node

```bash
ros2 run turtlesim turtlesim_node
```

**Expected output**:
```
[INFO] [turtlesim]: Starting turtlesim with node name /turtlesim
[INFO] [turtlesim]: Waiting for external commands on topic /turtle1/cmd_vel
```

You should see a window with a blue grid and an orange turtle in the center:

```
┌──────────────────────────────────────┐
│                                      │
│                                      │
│              grid background         │
│                                      │
│            turtle in center (🐢)     │
│                                      │
│                                      │
└──────────────────────────────────────┘
```

**Leave this terminal running!** The turtlesim_node process must stay active.

### Terminal 2: Control the Turtle

Open a **new terminal** (keep Terminal 1 running) and use the keyboard teleop node:

```bash
ros2 run turtlesim turtle_teleop_key
```

**Expected output**:
```
Reading from keyboard
---------------------------
Moving around:
        w
      a   d
        s

q/z : increase/decrease max speed by 10%
e/c : increase/decrease only linear speed by 10%
r/v : increase/decrease only angular speed by 10%

anything else : stop

q (quit)
```

Now use your keyboard:
- **w**: Move forward
- **s**: Move backward
- **a**: Turn left (counter-clockwise)
- **d**: Turn right (clockwise)
- **q**: Quit

Try moving the turtle around. You should see it move in the window and leave a trail!

---

## Step 7: Verify ROS 2 Communication

Keep both terminals running and open a **third terminal** to verify the ROS 2 system:

### List All Nodes

```bash
ros2 node list
```

**Expected output**:
```
/turtlesim
/teleop_turtle
```

Two nodes are running: the simulator and the teleop controller.

### List All Topics

```bash
ros2 topic list
```

**Expected output**:
```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

**Meaning**:
- `/turtle1/cmd_vel`: Where teleop_turtle publishes velocity commands
- `/turtle1/pose`: Where turtlesim publishes turtle position/heading
- `/turtle1/color_sensor`: Where turtlesim publishes the color under the turtle

### Watch Topic Data in Real-Time

Publish some commands with Terminal 2 (press 'w' to move forward) and watch the data flow:

```bash
ros2 topic echo /turtle1/cmd_vel
```

**Expected output** (while moving):
```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

This shows the velocity command being sent. Each `---` separator indicates a new message.

Press Ctrl+C to stop echoing.

---

## Concept 3: ROS 2 Package Structure (Preview)

For Lesson 4, you'll create your own ROS 2 package. Here's the structure:

```
my_robot_workspace/
├── src/
│   └── my_first_node/
│       ├── my_first_node/
│       │   └── __init__.py
│       │   └── publisher_node.py
│       ├── setup.py
│       ├── setup.cfg
│       ├── package.xml
│       └── resource/my_first_node
└── install/
└── build/
```

**Key files**:
- `package.xml`: Metadata (package name, version, dependencies)
- `setup.py`: Python package configuration
- `my_first_node/publisher_node.py`: Your actual node code

We'll create this structure in Lesson 4. For now, just understand the layout.

---

## Concept 4: Understanding Dependencies

ROS 2 has many dependencies. The most important for this chapter:

| Package | Purpose | When You'll Use It |
|---------|---------|-------------------|
| `rclpy` | Python ROS 2 library | Every node you write |
| `geometry_msgs` | Twist, Pose, Vector3 messages | Movement commands |
| `std_msgs` | String, Int32, Float64, Bool | Simple message types |
| `sensor_msgs` | Image, LaserScan, PointCloud2 | Sensor data |
| `nav_msgs` | Odometry, MapMetaData | Navigation data |

These are pre-installed in `ros-humble-desktop`. When we create custom packages, we'll specify dependencies in `package.xml`.

---

## Self-Assessment Checklist

Before moving to Lesson 3, verify:

- [ ] I installed ROS 2 Humble successfully
- [ ] `ros2 --version` shows "ROS 2 Humble"
- [ ] I launched Turtlesim and saw the turtle window
- [ ] I controlled the turtle with keyboard commands (w/a/s/d)
- [ ] `ros2 node list` shows `/turtlesim` and `/teleop_turtle`
- [ ] `ros2 topic list` shows `/turtle1/cmd_vel` and `/turtle1/pose`
- [ ] I understand that ROS 2 relies on environment variables (ROS_DISTRO, etc.)
- [ ] Python 3.10+ is installed (`python3 --version`)

If any checks failed, review the troubleshooting section below.

---

## Troubleshooting (Common Installation Issues)

### Error: "ros2: command not found"

**Cause**: ROS 2 setup.bash hasn't been sourced.

**Solution**:
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

If this works, add sourcing to your `.bashrc` (see Step 4).

---

### Error: "E: Could not get lock /var/lib/apt/lists/lock"

**Cause**: Another apt process is running (maybe Ubuntu's automatic updates).

**Solution**: Wait 5 minutes or reboot, then try again:
```bash
sudo apt update
```

---

### Error: "Unable to locate package ros-humble-desktop"

**Cause**: ROS 2 repository not added properly (Step 2).

**Solution**: Re-run Step 2 carefully:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt update
```

Then install again:
```bash
sudo apt install -y ros-humble-desktop
```

---

### Error: "Turtlesim window won't open" or "QXcbConnection: Could not connect to display"

**Cause**: Display server issue (common in VMs or WSL2).

**Solution for WSL2**:
```bash
export DISPLAY=$(awk '/nameserver / {print $2; exit}' /etc/resolv.conf):0
```

Add this to your `.bashrc` for WSL2.

**Solution for VirtualBox**: Ensure 3D acceleration is enabled in VM settings and use an adequate video memory (128MB+).

---

### Error: "No module named 'rclpy'" in a Python script

**Cause**: ROS 2 environment not sourced when running the script.

**Solution**: Either:
1. Source ROS 2 before running: `source /opt/ros/humble/setup.bash && python3 script.py`
2. Add sourcing to your script: `import subprocess; subprocess.run(['source', '/opt/ros/humble/setup.bash'], shell=True)`

---

### Error: "ros2 node list" shows no nodes

**Cause**: Turtlesim wasn't launched, or it crashed.

**Solution**:
1. Check Terminal 1 (where you ran turtlesim_node)
2. Look for error messages
3. Re-launch turtlesim: `ros2 run turtlesim turtlesim_node`

---

### Network Isolation (ROS_DOMAIN_ID)

**Issue**: Your ROS 2 topics conflict with another ROS 2 system on the same network.

**Solution**: Set a unique domain ID before launching nodes:
```bash
export ROS_DOMAIN_ID=1
ros2 run turtlesim turtlesim_node
```

Use `ROS_DOMAIN_ID=0` (default) in this course; change it only if you have multiple ROS 2 systems running.

---

### Slow Installation on Windows with WSL2

**Cause**: Disk I/O limitation or antivirus interference.

**Solution**:
1. Exclude WSL directories from antivirus scanning
2. Run Ubuntu WSL in a native WSL directory (not /mnt/c): `cd ~`
3. Use `sudo apt install` instead of GUI installers

---

### "Permission denied" when sourcing setup.bash

**Cause**: File permissions issue.

**Solution**:
```bash
chmod +x /opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash
```

---

## Further Exploration (Optional)

1. **ROS 2 Installation Docs**: https://docs.ros.org/en/humble/Installation.html
2. **Turtlesim Documentation**: https://docs.ros.org/en/humble/Tutorials/Turtlesim/Introducing-Turtlesim.html
3. **ROS 2 Cheat Sheet**: https://ubuntu.com/tutorials/ros-2-humble-cheat-sheet
4. **Environment Setup Guide**: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html

---

## What's Next?

In **Lesson 3: Nodes and Communication Patterns**, you'll:
- Learn how nodes, topics, and messages relate to each other
- Understand publish-subscribe vs. service patterns
- Use ROS 2 CLI tools to inspect running systems
- Build a conceptual model of ROS 2 architecture

**Estimated time**: 60 minutes

**Prerequisites for Lesson 3**: ROS 2 environment working, Turtlesim functional

---

## Layer 2: AI Collaboration Prompt

**For students working with Claude or another AI assistant:**

> "I just installed ROS 2 and launched Turtlesim. Ask Claude: 'What's happening behind the scenes when I press 'w' on my keyboard? How does the keyboard input eventually make the turtle move?'"

**Why this matters**: Understanding the data flow from input → teleop node → turtlesim node builds intuition for ROS 2 message passing.

---

## Concept Count & CEFR Validation

**Total A2 Concepts Introduced**: 7

1. Ubuntu 22.04 and Linux prerequisites
2. ROS 2 Humble installation and verification
3. Environment sourcing (setup.bash)
4. ROS environment variables (ROS_DISTRO, ROS_DOMAIN_ID)
5. Turtlesim simulator and keyboard teleoperation
6. ROS 2 CLI tools (ros2 run, ros2 node list, ros2 topic list)
7. ROS 2 package structure (preview for Lesson 4)

**CEFR A2 Alignment**: ✓ Step-by-step procedural learning, clear prerequisites, no abstract concepts

---

**Chapter 1, Lesson 2 Complete**

Next: Lesson 3 — Nodes and Communication Patterns (60 min)
