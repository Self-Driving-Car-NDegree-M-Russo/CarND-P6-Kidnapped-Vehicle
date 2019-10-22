## "Kidnapped" Vehicle Project


The goal of this project is the implementation, in C++, of an Particle Filter capable of localizing a vehicle using as input noisy sensor measurements and a feature map of the environment, with no "a priori" information (hence the "Kidnapped" situation).  

---
## Data Input

The data source for this Filter will be the Udacity [simulator](https://github.com/udacity/self-driving-car-sim/releases). 

#### The Map

The simulator will load a feature map described through the `map_data.txt` file that can be found in the `data` directory. This file includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id



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

