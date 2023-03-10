# v3.4.2
The unitree_legged_sdk is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.
* amarco added: https://github.com/unitreerobotics/unitree_legged_sdk/tree/3.4.2
* amarco: Checkout a specific tag as
```bash
git checkout -b <New Branch Name> <TAG Name>
git checkout -b dev_amarco v3.4.2
```


### Notice
support robot: Go1

not support robot: Laikago, Aliengo, A1. (Check release [v3.3.1](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1) for support)

### Sport Mode
Legged_sport >= v1.32

### Dependencies
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)
```bash
cd lcm-x.x.x
mkdir build
cd build
cmake ../
make
sudo make install
```

### Build
```bash
mkdir build
cd build
cmake ../
make
```

### Usage
Run examples with 'sudo' for memory locking.


---
/amarco/


### General notes
1. This folder is not meant to be a ROS (catkin) package, but simply a library that collects unitree's headers and source files used to communicate with the robot by the other ROS package: `unitree_legged_real`. Because it's not a ROS catkin package, `catkin_make` won't compile anything inside.
2. However, this folder contains the python bindings that wrap Unitree's communication files into a single Python module called `real_robot_interface_go1` that can be imported from any Python script outside this folder. These Python bindings have to be compiled manually. See instructions below.
3. This folder is ALSO a Python module by itself (nothing to do with the Python bindings mentioned above). It contains some useful toosl for data parsing. See instructions below about how to use it.

NOTE: The Python bindings can only be compiled on linux/windows architectures, but not on apple machines because it links against `./lib/libunitree_legged_sdk_aXX64.so`, which is a pre-compiled shared library provided by Unitree; we have no control over it.


### (2) Compiling Python module `real_robot_interface_go1`
This repository includes the software necessary to create the Python wrapper/bindings/module; it's called `pybind11`; it's located at `./python_interface_go1/pybind11` and can be donwnloaded as `git clone -b stable git@github.com:pybind/pybind11.git`. The `CMakeLists.txt` needs to find it. We do that by having the following lines:
```makefile
# amarco
add_subdirectory(python_interface_go1/pybind11)
pybind11_add_module(real_robot_interface_go1 python_interface_go1/src/real_robot_interface_go1.cpp)
target_link_libraries(real_robot_interface_go1 ${EXTRA_LIBS})
```
To compile it:
```bash
cd <root/of/this/repo>
mkdir build
cd build
cmake ..
make
```
The Python module can be found at `./build/real_robot_interface_go1.cpython-39-x86_64-linux-gnu.so`
Now, we need to tell Python where the module is located so that it can find it when importing it
```bash
export PYTHONPATH=$PYTHONPATH:/home/ubuntu/mounted_home/work/code_projects_WIP/unitree_legged_sdk_from_inside_robot/build
```
Finally, to import it on a Python script, we do
```Python
from real_robot_interface_go1 import RealRobotInterfaceGo1
```

##### Possible errors:
1. `ImportError: dynamic module does not define module export function (PyInit_real_robot_interface_go1)`
**Solution:** 


### (3) Adding this entire folder as a Python module (nothing to do with the Python bindings described above)
/TODO!!/
`unitree_legged_sdk_python_tools`
Run
```bash
pip install -e .
```
Then, in any other python file, outside this repository, do:
```python
from unitree_legged_sdk_python_tools.utils.data_parsing import read_cvs_file
```

