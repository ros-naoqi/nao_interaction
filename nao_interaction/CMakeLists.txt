cmake_minimum_required(VERSION 2.8.3)
project(nao_interaction)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find all required modules for compilation
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    sensor_msgs
    geometry_msgs
    nao_msgs
)

find_package(Boost QUIET COMPONENTS program_options signals)

# This will install python libraries
catkin_python_setup()

catkin_package()

#And this will install python nodes
install(DIRECTORY nodes/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS)

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

# Install config file (& ignore dynamic reconfigure files)
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    PATTERN "*.cfg" EXCLUDE)
