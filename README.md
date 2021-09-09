# Sample Code

## Problem Specification and Solution

Information on the problem specification and solution can be found in the included report: **report.pdf**

## Recommended Installation

- ROS Noetic 

- Ubuntu 20.04

## Getting Started

The project consists of a simulator, gps publisher, accelerometer publisher and estimator. To start system:

```bash
roscore
rosrun sample_code gps_publisher
rosrun sample_code accel_publisher
rosrun sample_code estimator
rosrun sample_code simulator
```

To view measurements and state estimate

```bash
rosrun rqt_plot rqt_plot
```

Accelerometer measurements, gps measurements and the state estimate can be viewed on the topics `/gps`, `/accel`, and `/estimator` respectively. 
