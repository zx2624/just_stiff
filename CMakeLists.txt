cmake_minimum_required(VERSION 2.8.3)
project(stiff_depth)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

set(PACKAGE_DEPENDENCIES
  
  eigen_conversions
  geometry_msgs
  message_runtime
  nav_msgs
  rosbag
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
  #sensor_driver
  #stiff_water_msg
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(ivcommon REQUIRED)

find_package(sensor_driver REQUIRED)
set(BUILD_SHARED_LIBS OFF)
include("${IVCOMMON_CMAKE_DIR}/functions.cmake")
google_initialize_ivcommon_project()

install(DIRECTORY launch DESTINATION share/stiff_depth/)

include(FindPkgConfig)
find_package(LuaGoogle REQUIRED)
find_package(PCL 1.7 REQUIRED)

#find_package(PCL 1.8 REQUIRED)
#set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)

#install(DIRECTORY launch DESTINATION share/covgrid_slam_ros/)
#install(DIRECTORY rviz_cfg DESTINATION share/covgrid_slam_ros/)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS src
)
include_directories(SYSTEM  ${catkin_INCLUDE_DIRS} ${ivcommon_INCLUDE_DIRS} ${LUA_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} src include)

add_executable(stiff_depth src/main.cpp )
target_link_libraries(stiff_depth PUBLIC ${catkin_LIBRARIES}  ${ivcommon_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS} ${sensor_driver_INCLUDE_DIRS} utils ivsensor glog gflags )

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} -std=c++11")
set_target_properties(stiff_depth PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(TARGETS stiff_depth
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS stiff_depth
  EXPORT stiff_depthExport#这是干嘛的
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
