# v3.4.2
The unitree_legged_sdk is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.
* amarco added: https://github.com/unitreerobotics/unitree_legged_sdk/tree/3.4.2

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

### Include pybind11
TODO: Inclide it in the git tree; till then:
1. Clone the repo directly into the folder:
```bash
git clone -b stable git@github.com:pybind/pybind11.git
```
2. Make sure that the following lines are added to the CMakeLists.txt:
```makefile
# amarco
add_subdirectory(pybind11)
pybind11_add_module(robot_interface python_interface.cpp)
target_link_libraries(robot_interface ${EXTRA_LIBS})
```
3. Compile
```bash
mkdir build
cd build
cmake ..
make
```


### Added Python package `unitree_legged_sdk_python_tools`
Run
```bash
pip install -e .
```
Then, in any other python file, outside this repository, do:
```python
from unitree_legged_sdk_python_tools.utils.data_parsing import read_cvs_file
```

