source install/setup.bash
colcon build --symlink-install --packages-select bot_msg routing csv_map canbus_cq control planning pointcloud_preprocess \
    rslidar_msg rslidar_sdk ground_filter rtk transform costmap_generator local_record remote_controller --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON