package_name=("calibration" "map" "routing" "localization" "perception" "HMI" "planning" "control" "canbus" "prediction" "transform" "bot_msg" "launch_manager")
cd src  
for element in ${package_name[@]}; do
    ros2 pkg create "$element" --build-type ament_cmake
done