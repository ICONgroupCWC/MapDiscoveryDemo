# Maze Discovery using Multiple Robots via Federated Learning

This repository showcases an implementation of FL in the context of maze exploration using LiDAR sensors-equipped robots. The main objective is to train classification models capable of accurately identifying grid area shapes within two square mazes with irregular walls. By leveraging FL, the robots can collaborate and share knowledge, enabling them to navigate and discover mazes more effectively. This repository contains the code and resources necessary to replicate and build upon this FL-based maze discovery system.
- There are three main phases in this work
  - Data collection phase
  - Local and FL based training phase
  - Inference and maze discovery phase


## System Architecture

The system architecture of this demonstration comprises of hardware, software and a robot platform.


## Hardware

These are the hardware components used in the system:
- Two JetBots ROS AI Kit powered by Nvidia Jetson nano developer module with 16GB eMMC and 4GB RAM.
- Jetson Nano developer module with ROS.
- Lenavo Thinkpad with ROS.

### Robot Platform


The demo setup consists of two 4x4 square grid mazes with two types of unique irregular shaped walls as the walls in two mazes. Paths from one grid center to other grid center is marked with white lines to guide the JetBots accurately between grid centers. Additionally, blue lines are marked in front of each grid center, enabling the robots to stop precisely at the center.


### Mobile Robots

The mobile robots used in this work are based on the open-source "JetBot ROS AI kit", which is built on NVIDIA Jetson Nano. Each robot is equipped with a 360-degree laser ranging LiDAR for observing the surrounding from the middle of the grid, along with an 8MP 160-degree field of view camera to precise navigation from one grid center to another.

For the setup and configuration of JetBot, please refer to the [official page](https://www.waveshare.com/wiki/JetBot_ROS_AI_Kit).


### Software

The Git repository contains the four ROS packages required to reproduce this work. Packages "data_collect", "line_following" and "maze_discovery" are needed in the JetBots, and the package "draw_maze" is required in the servers to visualize the maze discovery.

Apart from these ROS packages, a simple feed-forward neural network training script was used for local training, and a tool developed by ICON was used for the FL training part. To get the trained models in two mazes and the FL model, please contact any of the [contributors](#contributors).

#### Data Collection
