# Robot Mindset

Welcome to the **Robot Mindset** repository! This document will guide you through the setup process to get your development environment ready using a dev container.

---

## Table of Contents

- [Overview](#overview)
- [Setup Instructions](#setup-instructions)
  - [1. Open in a Dev Container](#1-open-in-a-dev-container)
  - [2. Install Dependencies](#2-install-dependencies)
  - [3. Source ROS Environment](#3-source-ros-environment)
  - [4. Build the Project](#4-build-the-project)
  - [5. Run the Node](#5-run-the-node)
- [Submodule: dev-setup](#submodule-dev-setup)
- [Additional Information](#additional-information)

---

## Overview

This repository, **Robot Mindset**, includes a submodule named [dev-setup](https://github.com/ipa-mdh/dev-setup/tree/master) that contains the necessary scripts and configuration to set up the development environment. The setup process leverages a development container to isolate dependencies and ensure consistency across different setups.

---

## Setup Instructions

### 1. Open in a Dev Container

Make sure you have the appropriate dev container configuration ready. Open the entire repository in your development environment (such as VS Code) that supports dev containers.

### 2. Install Dependencies

1. Open a terminal inside the dev container.
2. Navigate to the workspace folder (typically `/workspace`).
3. Run the installation script provided in the submodule by executing:

   ```bash
   bash ./src/robot_mindset/dev-setup/install.sh
   ```

This script will install all required dependencies for the project.

### 3. Source ROS Environment

Before building the project, source the ROS 2 Humble setup file to ensure all ROS environment variables are available:

```bash
source /opt/ros/humble/setup.bash
```

### 4. Build the Project

Build the project using `colcon`. Navigate to the root of your workspace (e.g., `/workspace`) and run:

```bash
colcon build --symlink-install
```

Once the build is complete, source the local setup file to overlay the freshly built packages:

```bash
source install/setup.bash
```

### 5. Run the Node

Now you can run the ROS node using the following command:

```bash
ros2 run robot_mindset node
```

---

## Submodule: dev-setup

The submodule `dev-setup` contains scripts and configurations for setting up the development environment. Ensure that this submodule is properly initialized and updated. To initialize and update submodules (if not done already), run:

```bash
git submodule update --init --recursive
```

---

## Additional Information

- **Dev Container**: Utilizing a dev container ensures that all developers work with the same configuration and software versions. If you encounter issues, verify your dev container configuration.
- **Dependencies**: The installation script (`install.sh`) handles dependencies, but you may need to adjust certain configurations based on your specific development environment.
- **ROS 2**: This project is built on ROS 2 Humble. Ensure that your system supports ROS 2 Humble or adjust accordingly if using a different version.

For further help, refer to the ROS 2 and dev container documentation or contact the project maintainers.

Happy coding!