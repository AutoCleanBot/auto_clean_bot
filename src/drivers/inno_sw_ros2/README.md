# Inno Lidar Ros2 Usage Document

## About the project
  The project is based on the innovusion client sdk and supports all types of lidars for innovusion.After the project is started, it will monitor the UDP data of Lidar and use topic:/iv_points to publish point cloud data. This project serves as a demo for customers to refer to how to use innovusion client sdk on ros.

  **directory structure**



  ```shell
  ├── LICENSE
  ├── build.bash
  ├── README.md
  └── src
      └── inno_lidar_ros
          ├── CMakeLists.txt
          ├── launch
          │   └── ivu_pc2.py
          ├── package.xml
          ├── shm_profile.xml
          └── src
              ├── inno_sdk
              └── node
                  ├── driver
                  │   ├── driver_lidar.cc
                  │   ├── driver_lidar.h
                  │   └── point_types.h
                  └── publisher.cc
  ```


## Environment and Dependencies

The following official versions are verified to support, of course **Rolling distribution** is also supported, we are also compatible with docker ros2

| Distro                                                       | Release date   | Logo                                                         | EOL date      |
| ------------------------------------------------------------ | -------------- | ------------------------------------------------------------ | ------------- |
| [Humble Hawksbill](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html) | May 23rd, 2022 | ![Humble logo](https://docs.ros.org/en/humble/_images/humble-small.png) | May 2027      |
| [Galactic Geochelone](https://docs.ros.org/en/humble/Releases/Release-Galactic-Geochelone.html) | May 23rd, 2021 | ![Galactic logo](https://docs.ros.org/en/humble/_images/galactic-small.png) | November 2022 |
| [Foxy Fitzroy](https://docs.ros.org/en/humble/Releases/Release-Foxy-Fitzroy.html) | June 5th, 2020 | ![Foxy logo](https://docs.ros.org/en/humble/_images/foxy-small.png) | May 2023      |

## Build

- **Build** package

  ```bash
  ./build.bash
  ```

## Configuration
| Parameter          | Default Value | description   |
|:--------:          | :---------:   | :---------:   |
| packets_mode       |  false        |  packets topic flag   |
| aggregate_num      |  10           |  aggregate packets num   |
| replay_rosbag      |  false        |  replay rawPacket rosbag flag   |
| frame_id           |  innovusion   |      -        |
| lidar_name         |  falcon       |      -        |
| device_ip          |  172.168.1.10 |  lidar ip     |
| output_topic       |  iv_points    |  topic name   |
| port               |  8010         |   tcp port    |
| udp_port           |  0            |  If < 0, use tcp to send, If = 0, use lidar configured udp_port, If > 0, set the value to udp_port   |
| pcap_file          |  ""           |  path of path playback pcapfile, If empty, use real lidar   |
| packet_rate        |  20           |  file playback rate                 |
| file_rewind        |  0            |  number of file replays 0:no rewind -1: unlimited times             |
| reflectance_mode   |  true         |  false:intensiy mode true:reflectance_mode mode |
| multiple_return    |  1            |  lidar detection echo mode        |
| coordinate_mode    |  1            |  convert the xyz direction of a point cloud   |
| name_value_pairs   |  ""           |  some settings of lidar are consistent with the usage of inno_pc_client |
| max_range          |  2000.0       |  point cloud display maximum distance (unit:m)|
| min_range          |  0.4          |  point cloud display minimun distance (unit:m)|
| continue_live      |  1            |  reboot after an exception occurs  |
| lidar_log_limit    |  info         |  limit log from lidar, can choose from (info warn error)  |

## Run as publisher node
### **Activate** enviroment

  ```
  source install/setup.bash
  ```

  - (Optional)Enable ROS2 share memory

    ```bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=`pwd`/shm_profile.xml
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ```

### **Run**

  - ros2 **run**
  ```bash
  ros2 run innovusion <publisher>
  ```

  - ros2 **run with specify parameters**
  ```bash
  ros2 run innovusion <publisher> --ros-args -p <arg1>:=val1 -p <arg2>:=val2
  ```

  -  ros2 **launch**

  Connect the lidar and confirm the **lidar_ip** in `launch/ivu_pc2.py`, default(172.168.1.10)

  For parameter modification, please refer to `launch/ivu_pc2.py`


  ```bash
  ros2 launch innovusion ivu_pc2.py device_ip:=172.168.1.10
  ```


  -  **lidar roi**

  In order to facilitate real-time adjustment of roi data, the program will start two subscription nodes to subscribe to the latest roi data in real time. When the roi data needs to be adjusted, the following command can be used to publish the roi data to be set
  ```
  ros2 topic pub /hori_roi std_msgs/msg/Float64 data:\ <hori_roi_data>
  ros2 topic pub /vertical_roi std_msgs/msg/Float64 data:\ <vert_roi_data>
  ```
