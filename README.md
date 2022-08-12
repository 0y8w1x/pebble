## Pebble
For more information see https://gitlab.com/VladyslavUsenko/basalt

### Warning: This project is targeted to people who experimented with the original basalt. The code is given as is. I won't fix your issues, but I do answer your questions.

This project is a stripped down version of basalt. The goal is to have a deployable version of the basalt vio system, without any quality of life features. The idea is that you develop, test, and calibrate your system with the main basalt project, and then run this code on the target architecture. The goal is to have only those parts in pebble, that are crucial to have online slam working. In that sense, the offline slam system may or may not be broken.

Dependencies:
* eigen
* tbb
* basalt_headers
* sophus
* opencv (might be possible to remove)
* cereal (might be possible to remove)

Dependencies to test the code:
* librealsense2 (https://robots.uc3m.es/installation-guides/install-realsense2.html)

Directories that were removed (to understand this, please compare the directory structure to the basalt project linked above):
* test
* doc
* docker
* calibration

Dependencies that were removed (again, to understand this please get familiar with the original project):
* pangolin
* ros
* apriltag
* opengv
* fmt
* cli11
* magic_enum


## Installation
### Source installation for Ubuntu 20.04 (the machine I'm testing on). Other platforms might work aswell.testing on.
Clone the source code for the project and build it.
```
git clone --recursive https://github.com/0y8w1x/pebble.git
cd pebble
./scripts/install_deps.sh
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j8
```

## Testing
I tested the online slam system with a T265 camera by Intel. I've modified the original rs_t265_vio.cpp file to be as minimal as possible. That way it's easy to see what new cameras need to implement.

Some other cameras might be implemented in the future.

## Licence
The code is provided under a BSD 3-clause license. See the LICENSE file for details.
Note also the different licenses of thirdparty submodules.

Some improvements are ported back from the fork
[granite](https://github.com/DLR-RM/granite) (MIT license).
