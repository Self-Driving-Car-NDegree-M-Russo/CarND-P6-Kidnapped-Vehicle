## "Kidnapped" Vehicle Project


The goal of this project is the implementation, in C++, of a 2D Particle Filter capable of localizing a vehicle using as input noisy sensor measurements and a feature map of the environment, with no "a priori" information (hence the "Kidnapped" situation).  

The source code is contained in the [src](./src) folder in this git repo. It is the evolution of a starter project provided directly by Udacity, where the [particle_filter.cpp](./src/particle_filter.cpp) was modified. The other files have been left fundamentally unchanged.

The following sections of this writeup will provide details on the filter operations and the data flow, and in doing so the fundamental pieces of the code will be explained. A final [Results](Kidnapped_Vehicle_writeup.md#filter-results) section will show the outcomes of the filter running against the reference data set. 


---
## Data Input

The data source for this Filter will be the Udacity [simulator](https://github.com/udacity/self-driving-car-sim/releases). The compiled code will open a websocke session to the sim and provide localization information that will be shon on the screen.

### _The Map_

The simulator will load a feature map described through the [`map_data.txt`](./data/map_data.txt) file that can be found in the `data` directory. This file includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns, describing x position, y position and landmark id.

the map is read in the [main.cpp](./src/main.cpp) file (lines 40-45):

```sh
  // Read map data
  Map map;
  if (!read_map_data("../data/map_data.txt", map)) {
    std::cout << "Error: Could not open map file" << std::endl;
    return -1;
  }
```

the `read_map_data` function is defined in [helper_functions.h](./src/helper_functions.h) (lines 85-122).

## Initialization

The first thing that happens to the filter is to have its state initialized at the value of the first measurement ([main.cpp](./src/main.cpp), line 76).

```sh
     pf.init(sense_x, sense_y, sense_theta, sigma_pos);
```

This instruction initialize the filter starting with the (x,y,theta) collected through simulated GPS measurements. Specifically, the `init(...)` function is coded in [particle_filter.cpp] (./src/particle_filter.cpp) (lines 36-67) and in it a vector of particles is created around the measured position, considering the noise of the GPS measurement. 

## Prediction

```sh
    pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);
```

## Update Particle Weights

```sh
    pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
```

## Resampling

```sh
    pf.resample();
```

## Accuracy Evaluation



## Compiling the Code

After having cloned this repo and taken care of the dependencies outlined in the repo [README](./README.md), the main program can be built and ran by doing the following from the project top directory.

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

Once started, the Filter code will open a WebSocket connection trying to reach to a data source, and that will be the Udacity [simulator](https://github.com/udacity/self-driving-car-sim/releases).

## Filter Results

