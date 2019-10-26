# Particle Filter for Localization ("Kidnapped" Vehicle)
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---

This Project is submitted as part of the Udacity Self-Driving Car Nanodegree.

For it the goal is to write C++ code implementing a Particle Filter to loaclize a moving object of interest, starting from a feature map and noisy measurements.

The source code for this project is submitted as part of this Git repo (in the [src](/src) folder). A detailed explanation is provided in a separate [writeup](Kidnapped_Vehicle_writeup.md), that documents also the results of the filter.  

Dependencies
---
First of all, this project involves the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

The simulateor will communicate with the code through a websocket connection. To enable that, this repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Other important dependencies are:

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

Further tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).

## Compiling the Code

After having cloned this repo and taken care of the dependencies outlined before, the main program can be built and ran by doing the following from the project top directory.

```sh
mkdir build
cd build
cmake ..
make
./particle_filter
```

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

```sh
./clean.sh
./build.sh
./run.sh
```

Note that the first script is not strictly necessary for the first build, but is good practice to clean the project before subsequent builds.
