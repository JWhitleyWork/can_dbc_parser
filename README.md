# CAN DBC Parser

This is an implementation of a DBC parser which will represent messages and signals as C++
objects. This is nearly completely copied from https://github.com/NewEagleRaptor/raptor-dbw-ros2
with minor modifications (different namespace and a new CanFrame struct) but de-coupled from ROS.

## Build / Installation Instructions

From the repository folder:

```
mkdir build
cd build
cmake ..
sudo make install
```

## Uninstall Instructions

From the repository folder:

```
cd build
sudo make uninstall
```

## Usage Instructions

In your CMakeLists.txt:

```
target_link_libraries(MyTarget PRIVATE CanDbcParser::can_dbc_parser)
```
