p align="center">
  <a href="https://example.com/">
    <img src="https://via.placeholder.com/72" alt="Logo" width=72 height=72>
  </a>

  <h3 align="center">Assistive Teleop</h3>

  <p align="center">
    <br>
    A Modular ROS based package for robot that avoids obstacle in the path while being controlled with teleoperations or being fully autonomous 
    <br>
    <a href="https://github.com/atom-robotics-lab/assistive_teleop/issues/new?labels=bug">Report bug</a>
    ·
    <a href="https://github.com/atom-robotics-lab/assistive_teleop/issues/new?assignees=namikxgithub,kartik9250,jasmeet0915,insaaniManav&labels=enhancement">Request feature</a>
  </p>
</p>

## Table of contents

- [About our Project](#about-our-project)
- [Requirements](#requirements)
- [Usage & Installation](#usage&installation)
- [Samples](#samples)

## About our Project
  Assistive Teleop is a Ros package which helps simulate a invironment where a bot can be controlled with teleoperations or be fully autonomous while it avoids obstacles (both dynamic and static) in its path. Our team has made it so that it can easily be configured with your needs. This is modular package which, if required can also work with our other packages depending on the requirements of the project. It uses Lidar for assesment of the multiple obstacles in its path. Depending on their distance and corelation of their positions with our goal it classifies them as an obstacle.   

## Requirements

- ### Installing the dependencies

```bash
  
```


### Usage

- Open your terminal and go to your Ros workspace

```bash
  catkin_build && source devel/setup.bash
```
