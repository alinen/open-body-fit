# open-body-fit

Software for estimating biomechanics from poses extracted from video.
Send feedback and inquiries to `anormoyle @ brynmawr.edu`

## Demo

We demonstrate the software using a video of Nicaraguan sign language. 
The software takes the following input files (located in the `demo` directory).

Input Files:

* `demo/images` - frames of input video for checking the fit (30 fps)
* `demo/036CR1-3d.csv` - extracted poses from video (csv file)

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

First, build the dart dependency. 

```
cd open-body-fit/src/external/dart
mkdir build
cd build
cmake ..
make -j6
```

Then, return to the root directory and build the open-body-fit utility.

```
cd open-body-fit
mkdir build
cd build
cmake ..
make -j6
../bin/open-body-fit
```

## Cite

Normoyle A., Artacho B., Savakis A., Senghas A., Badler N. I., Occhino C., Rothstein S. J., and Dye M. W. G. Dye, 2022, _Open-Source Pipeline for Skeletal Modeling of Sign Language Utterances from 2D Video Sources [Talk]_, Theoretical Issues in Sign Language Research (TISLR)

[You can see the talk here.](https://www.youtube.com/watch?v=q2bP6BvWoEo) 
