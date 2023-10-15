# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
 
# Source the base workspace, if built
if [ -f /home/ws/install/setup.bash ]
then
  source /home/ws/install/setup.bash
fi
