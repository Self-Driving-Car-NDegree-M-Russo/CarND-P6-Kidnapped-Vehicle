## "Kidnapped" Vehicle Project


The goal of this project is the implementation, in C++, of an Particle Filter capable of localizing a vehicle using as input noisy sensor measurements and a feature map of the environment, with no "a priori" information (hence the "Kidnapped" situation).  

---
## Data Input

The data source for this Filter will be the Udacity [simulator](https://github.com/udacity/self-driving-car-sim/releases).



## Compiling the Code

The code is intended to be compiled using CMake and Make, through some shell script provided. After having cloned this repo and taken care of the dependencies outlined in the repo [README](./README.md), you should just need to run, from the root folder: 

```sh
./clean.sh
```

Followed by

```sh
./clean.sh
```

Note that the first is not strictly necessary for the first build, but is good practice to clean the project before subsequent builds.

## Filter Results

Once the code is compiled it can be easily run through the command

```sh
./run.sh
```

typed again from the root folder.

The Filter code will open a WebSocket connection trying to reach to a data source, and that will be the Udacity [simulator](https://github.com/udacity/self-driving-car-sim/releases).
