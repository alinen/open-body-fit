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

You will need [cmake](https://cmake.org) and C++ development tools installed to build from source.

### Ubuntu

Tested with gcc.

```
mkdir build
cd build
cmake ..
make
```

## Cite

Normoyle A., Artacho B., Savakis A., Senghas A., Badler N. I., Occhino C., Rothstein S. J., and Dye M. W. G. Dye, 2022, _Open-Source Pipeline for Skeletal Modeling of Sign Language Utterances from 2D Video Sources [Talk]_, Theoretical Issues in Sign Language Research (TISLR)

[You can see the talk here.](https://www.youtube.com/watch?v=q2bP6BvWoEo) 
