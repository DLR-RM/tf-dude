# tf dude #
[![Build Status](https://rmc-jenkins.robotic.dlr.de/jenkins/buildStatus/icon?job=indoor-map%2Ftf_dude%2Fmaster)](https://rmc-jenkins.robotic.dlr.de/jenkins/job/indoor-map/job/tf_dude/)
[![Current Version](https://img.shields.io/badge/current%20version-0.0.13-blue.svg)](https://rmc-github.robotic.dlr.de/rollin-justin/tf_dude/releases)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue)
![Python](https://img.shields.io/badge/Python-3.6-blue)


> I have a dude for everything. There is the 300 inch-TV dude, there is the ticket dude, there is dude for a suit.
>
> -Barney Stinson, more or less

But who you gonna call (:ghost:) when asking for the transform between your robot's links or the world frame?
The answer is easy, you call the **tf dude**.

## Overview ##
*tf_dude*, as part of the *dude* framework, centralizes dependent transformations in a single forest of transformation trees.
The main purpose is to provide a simple yet powerful interface for accessing transformations from any process in the processing network.
Furthermore, it includes uncertainty estimations based on [Lie Algebra](https://en.wikipedia.org/wiki/Lie_algebra) to take into account tolerances, gravity impact or other sources of sensor noise.

## Roadmap ##
- *tf* library
  - [X] core functionality
  - [X] generic [IPC](https://en.wikipedia.org/wiki/Inter-process_communication)
  - [X] Client API (Functions for CRUD, Handles for Frames & Paths)
  - [X] Lie Algebra integration (currently no uncertainty support)
  - [ ] robust exception handling
  - [ ] Smart Caching of handle data
  - [ ] Temporal caching of frame data
  - [ ] event-based updates of client data
  - [X] Reduction of network-load
  - [ ] local-cluster high-speed integration
- *plugins*
  - [X] links_and_nodes implementation
  - [X] ROS implementation
- *tf_dude*
  - [X] simple CLI server app
  - [ ] init_tree integration
  - [ ] visualization

## How to use ##
Find out [in the documentation](doc/documentation.md).

# Development
## Toolchain File
For local development, you need to pass the toolchain file directly to CMake.
For CLion this can be automated using by adding the following CMake option in `File->Settings->Build...->CMake`
```
-DCMAKE_TOOLCHAIN_FILE=$CMakeProjectDir$/build/conan_paths.cmake
```

## Selecting the right plugin
The *tf_dude* library is independent on any IPC framework.
However, if you are developing, you might want to test your code somehow, for this you need to active the right plugin for your environment.
By default, *tf_dude* builds for ROS, but if you want another plugin, you have to activate the build:

For LN add the following to your build seetings (also deactivating ROS/ROS2 for less build stuff)
```
-DBUILD_MODULE_TF_PLUGINS_LN=ON
-DBUILD_MODULE_TF_PLUGINS_ROS=OFF
-DBUILD_MODULE_TF_PLUGINS_ROS2=OFF
```

The last thing missing is setting the default plugin to load
```
-DDEFAULT_PLUGIN=ln
```

## Creating a dev environment using conan
Depending on the IPC you are using, you can create your environment using (in the root of the repository)
```
conan install conanfile.tf_dude.<IPC>.py -if <BUILD_FOLDER> -g virtualenv [-u]
```

for example for ROS it is
```
conan install conanfile.tf_dude.ros.py -if <BUILD_FOLRDER> -g virtualenv [-u]
```

some IPC implementation split the message definition and the compilation into two packages, e.g. LN, here you first have to build the definitions 
```
cissy create conanfile.ln_msg_defs.py -rv None
```
Note the `-rv None` for creating a revision with the name `None`
