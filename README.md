# open-body-fit

Software for estimating biomechanics from poses extracted from video. This repository contains C++ code for visualizing and 
generating estimates of velocities, accelerations, forces, etc from points extracted from a video. We additionally include our 
R scripts for visualizing and analyzing this data. 

[Click here for documentation on our R scripts](https://alinen.github.io/open-body-fit/R/036CR1-Example.html)

Please send feedback and questions to `anormoyle @ brynmawr.edu`

[![ubuntu](https://github.com/alinen/open-body-fit/actions/workflows/cmake-linux.yml/badge.svg)](https://github.com/alinen/open-body-fit/actions/workflows/cmake-linux.yml)

![Peek 2022-09-26 16-09](https://user-images.githubusercontent.com/259657/192370359-41aeeacb-7542-40e1-a6ea-72fe0e56c79a.gif)

## Demo

We demonstrate the software using a video of Nicaraguan sign language. 
The software takes the following input files (located in the `demo` directory).

Input Files:

* `demo/images` - frames of input video for checking the fit (30 fps)
* `demo/036CR1-3d.csv` - extracted poses from video (csv file)

Keyboard controls:

* Press '3' to switch from 3D and 2D views
* Press SPACE to play/pause the animation
* When paused, press Up to go forward one frame 
* When paused, press Down to go backward one frame
* Press 'M' to toggle the physics visualization (yellow cubes)
* Press 'S' to toggle the skeleton (blue)
* Press 'P' to toggle the points (black spheres)
* Press 'H' to center the view on the head
* Press 'W' to center the view on the wrist

Camera controls (3D view):

* Left mouse button rotates the camera
* Middle mouse button zooms in and out
* Right mouse button pans the camera

## Build from source

You will need [cmake](https://cmake.org) and C++ development tools installed to
build from source.

The repository includes the source for the following dependencies 

* [dart](https://dartsim.github.io/) - physics
* [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) - physics math
* [levmar](http://users.ics.forth.gr/~lourakis/levmar/) - optimization
* [imgui](https://github.com/ocornut/imgui) - gui
* [agl](https://github.com/alinen/agl) - graphics
* [atk](https://github.com/alinen/atk) - animation

### Ubuntu

**Install dependencies**

```
sudo apt-get install libglfw3-dev mesa-utils libglew-dev
sudo apt-get install f2c libblas-dev liblapack-dev
sudo apt-get install libtinyxml-dev libeigen3-dev 
sudo apt-get install libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev
```

NOTE: If you receive an error involving `boost_filesystem`, also do `sudo apt-get install libboost-dev-all`.

**Build source**

From the root directory (e.g. open-body-fit), build dart, other dependencies, and our code. 

```
cmake -S ./src/external/dart -B ./src/external/dart/build -DCMAKE_BUILD_TYPE=Debug
cmake --build ./src/external/dart/build  --config Debug
cmake -B ./build -DCMAKE_BUILD_TYPE=Debug
cmake --build ./build  --config Debug
```

**Run**

The executable should be run from the `/bin` directory

```
cd open-body-fit/bin
./open-body-fit
```

## Cite

Normoyle A., Artacho B., Savakis A., Senghas A., Badler N. I., Occhino C., Rothstein S. J., and Dye M. W. G., 2022, _Open-Source Pipeline for Skeletal Modeling of Sign Language Utterances from 2D Video Sources [Talk]_, Theoretical Issues in Sign Language Research (TISLR)

[You can see the talk here.](https://www.youtube.com/watch?v=q2bP6BvWoEo) 
