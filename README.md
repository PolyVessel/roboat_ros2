# roboat-ros2

### Packages

- `roboat_interfaces` - All the Message and Service definitions
- `roboat_perception` - Nodes gathering information from the environemnt GPS, Radio, etc.

### Development Environment

This repository contains a .devcontainer folder which describes the developer environment. This should setup all the dependencies and source the ROS2 environment.

##### Prerequisites

- Git
- Docker Desktop
- VSCode with DevContainer Extension

##### Starting the Dev Environment

In the bottom left of VSCode, click on the "Open a Remote Window" button (looks like two arrows pointing to one another). Then click on Reopen in Container. The Dev Container should build and startup.

##### Building the Project

In order to build the project from the Command Pane, use Tasks: Run Task > colon:build


##### Testing the Project

In order to test the project from the Command Pane, use Tasks: Run Task > colon:test