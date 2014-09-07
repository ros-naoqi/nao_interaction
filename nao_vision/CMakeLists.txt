cmake_minimum_required(VERSION 2.8.3)
project(nao_vision)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find all required modules for compilation
find_package(catkin)

catkin_package()

#And this will install python nodes
install(DIRECTORY nodes/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS)
