cmake_minimum_required(VERSION 3.0.2)
project(liantong_v3)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/bridge_node.py scripts/scan_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
