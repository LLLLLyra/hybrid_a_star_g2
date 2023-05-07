# Hybrid A* Planing with G2 Continuity

## Introduction

This repository is derived from *Open Space Planer submodule, Apollo planning module*, replacing the original G1 exploring and RS curves with G2.

## Prerequisite

Before compiling, we should do some preperation.

- Install dependencies.

    - glog

        ```bash
        cd ~ 
        mkdir Source
        cd Source
        git clone https://github.com/google/glog.git
        sudo apt-get install autoconf automake libtool
        cd glog
        mkdir build
        cmake ..
        make
        sudo make install
        ```    

    - protobuf
  
        ```bash
        cd ~/Source
        sudo apt-get install autoconf automake libtool curl make g++ unzip
        wget  https://github.com/protocolbuffers/protobuf/releases/download/v3.7.1/protobuf-cpp-3.7.1.tar.gz
        tar -xzvf protobuf-cpp-3.7.1.tar.gz
        cd protobuf-3.7.1
        ./autogen.sh
        ./configure
        make
        make check
        sudo make install
        sudo ldconfig
        ```

    - osqp

        ```bash
        cd Source
        git clone --recursive https://github.com/oxfordcontrol/osqp
        cd osqp
        mkdir build && cd build
        cmake -G "Unix Makefiles" ..
        cmake --build .
        sudo make install
        ```
    
## G2 Continuity

Different from the original Apollo with Bicycle Model exploring new nodes and connecting with Reed Shepp curves, we use Clothoid curves for exploration and impove Reed Shepp curves with continuous curvature. More details, please refer [here](https://github.com/hbanzhaf/steering_functions).

To fit the main skeleton of Apollo, we add a new feature, `kappa`, to `Node3d`, and change the method `Next_node_generator` in `HybridAStar` to `Next_node_generator_clothoid`. 

Denote $\kappa$ as the curvature of a node and $\delta$ as the steer wheel angle. There are three scenarios in total:

- Move towards a straight line;

  $\delta \rightarrow 0$ && $\kappa \rightarrow 0$
  
- Move towards a circle;

  $\delta \rightarrow \delta_{max}$ && $\kappa \rightarrow \kappa_{max}$

- Move towards a Clothoid curve.

  $0< \delta < \delta_{max}$

We use the following ODEs for Clothoid exploring

$$\begin{cases}
d \kappa = A ds \\
d \theta = \kappa (s) ds \\
dx = \cos \left(\theta(s)\right)ds \\
dy = \sin \left(\theta(s)\right)ds
\end{cases}$$
