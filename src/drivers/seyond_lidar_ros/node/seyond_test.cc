/*
 *  Copyright (C) 2024 Seyond Inc.
 *
 *  License: Apache License
 *
 *  $Id$
 */

#include <signal.h>
#include <memory>
#ifdef ROS_MAKE
#include <ros/ros.h>
#include <ros/package.h>
#include "src/test/ros1_test.hpp"
#elif ROS2_MAKE
#include <rclcpp/rclcpp.hpp>
#include "src/test/ros2_test.hpp"
#endif

static void shutdown_callback(int sig) {
#ifdef ROS_MAKE
  ros::shutdown();
#elif ROS2_MAKE
  rclcpp::shutdown();
#endif
}


int main(int argc, char *argv[]) {
#ifdef ROS_MAKE
  ros::init(argc, argv, "test", ros::init_options::NoSigintHandler);
#elif ROS2_MAKE
  rclcpp::init(argc, argv);
#endif

  signal(SIGINT, shutdown_callback);

  std::shared_ptr<ROSDemo> ros_driver_ptr = std::make_shared<ROSDemo>();
  ros_driver_ptr->init();
  ros_driver_ptr->spin();
  return 0;
}
