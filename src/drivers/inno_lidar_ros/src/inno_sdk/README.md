# Introduction
CLient_SDK project is the software development tool to help application developers easily interact with lidar(robinw). This tool supports multiple development languages, such as C++, C, python, R and so forth. And Rich development documentation([HOW_TO_USE_CLIENT_SDK.md](./docs/HOW_TO_USE_CLIENT_SDK.md)) and sample code([simple_get_pcd.cpp](./apps/example/simple_get_pcd.cpp)) are provided to allow developers to get quickly started. Besides, there are some useful execution files and scripts in ```apps``` directory. For detailed information, please refer to the corresponding documents in each directory.

# Lidar Supported
+ Robin_W
+ Robin_E
+ Falcon2.1
+ Falcon_K
+ Falcon_I

# Type of pointcloud supported
+ Spherical coordinate system
+ Cartesian coordinate system

# Third dependency libraries
Client SDK is supported by
  + MacOs
  + mingw
  + linux(ubuntu16.04, ubuntu18.04, ubuntu20.04)

Please press *[here](/docs/DEPENDENCY.md)* to see detail about third dependency libraries

# Clone


# Quick Compile
## for Linux and MacOS
```
cd build && ./build_unix.sh
```
## for windows

```
cd build && build_win.bat
```
You should configure the build_tools in advance.

MSVC free compiler *[download](!https://aka.ms/vs/17/release/vs_buildtools.exe)*

set ROOT_PATH=D:\win_build_package

# Directory Structure
+ ```apps```: Source files to generate applications
  + ```example```: Demo about how to get pointcloud. Please press *[here](./docs/demo.md)* to see detailed information.
  + ```pcs``` Pointcloud static linked executables. Please press *[here](./docs/inno_pc_client.md)* to see detailed information.
  + ```ros``` Source files to build ros deb package that can run in ROS
  + ```tools``` All kinds of scripts and excutive file
    + ```check_net``` Tool for network check. Please press *[here](./docs/check_net.md)* to see detailed information.
    + ```get_pcd```  Tool for obtaining pointcloud and converting file format. Please press *[here](./docs/get_pcd.md)* to see detailed information.
    + ```http_command_test``` Tool for http command's test. Please press *[here](./docs/http_command_test.md)* to see detailed information.
    + ```innovusion_wireshark_plugin``` Lua script for wireshark. Please press *[here](./docs/innovusion_lua.md)* to see detailed information.
    + ```latency_diagram``` Tool for calculating the pointcloud latency. Please press *[here](./docs/latency_diagram.md)* to see detailed information.
    + ```lidar_util``` lidar configuration tools can modify some configurations of lidar, get information of lidar and upload or download file (may cancle later). Please press *[here](./docs/innovusion_lidar_util.md)* to see detailed information.
    + ```parse``` Tool for parsing .pcap file. Please press *[here](./docs/parse.md)* to see detailed information.
    + ```py_module``` Tool for python language supported. Please press *[here](./docs/py_module.md)* to see detailed information.
+ ```docs``` Documents help compile and use client SDK
+ ```build``` All kinds of build script
+ ```lib``` Library files
+ ```CMakeLists.txt``` Compile script by CMake
+ ```Makefile``` Compile script by Make
+ ```README.md``` Introduction of Client SDK
+ ```src``` Source files to generate library files
  + ```sdk_client``` Source files to build sdk_client library
  + ```sdk_common``` Source files to build sdk_common library
  + ```utils``` Source files to build utility library



```shell
├── apps
│   ├── example
│   ├── pcs
│   └── tools
│       ├── check_net
│       ├── get_pcd
│       ├── http_command_test
│       ├── innovusion_wireshark_plugin
│       ├── latency_diagram
│       ├── lidar_util
│       ├── parse
│       └── py_module
├── build
├── docs
├── CMakeLists.txt
├── lib
├── Makefile
├── README.md
└── src
    ├── sdk_client
    ├── sdk_common
    └── utils
```
