cmake_minimum_required(VERSION 2.8.3)
project(octomap_world)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/octomap_world.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
