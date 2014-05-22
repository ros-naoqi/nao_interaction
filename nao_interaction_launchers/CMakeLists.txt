cmake_minimum_required(VERSION 2.8.3)
project(nao_interaction_launchers)

find_package(catkin REQUIRED)

catkin_package()

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

