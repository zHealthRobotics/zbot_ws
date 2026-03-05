This repository contains software stack for zbot (MVP Humanoid). 

source /opt/ros/humble/setup.bash
ros2 run xacro xacro \
/home/office/zbot_ws/src/zbot_description/urdf/zbot.urdf.xacro \
-o /home/office/zbot_ws/src/zbot_description/urdf/zbot.urdf
