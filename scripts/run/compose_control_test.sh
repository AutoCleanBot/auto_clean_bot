#! /bin/bash

source install/setup.bash

ros2 launch control control.launch.py

# 新开一个标签页, 运行local_record_test
gnome-terminal --tab --title="local_record_test" -- ros2 launch test local_record_test.launch.py

# 新开一个标签页, 运行routing
gnome-terminal --tab --title="routing" -- ros2 launch test routing.launch.py

# 新开一个标签页, 运行planning
gnome-terminal --tab --title="planning" -- ros2 launch test planning.launch.py



