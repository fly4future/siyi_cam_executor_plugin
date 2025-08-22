cmake_minimum_required(VERSION 3.15.0)
project(siyi_cam_executor_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPENDENCIES
  roscpp
  cmake_modules
  pluginlib
  iroc_mission_handler
  mrs_lib
  siyi_cam_driver
)

set(LIBRARIES
  IROCMissionHandler_SiyiCamExecutorPlugin
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wpedantic)

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(IROCMissionHandler_SiyiCamExecutorPlugin
  src/gimbal_plugin.cpp
)

add_dependencies(IROCMissionHandler_SiyiCamExecutorPlugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(IROCMissionHandler_SiyiCamExecutorPlugin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
