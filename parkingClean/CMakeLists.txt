cmake_minimum_required(VERSION 2.8.3)
project(parkingClean)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   ecl_threads
   tf
   geometry_msgs
   rospy
   std_msgs
   sensor_msgs
   client
   message_generation
   urdf
   move_base_msgs
   move_base
   actionlib_msgs
   actionlib
)


find_package( Ceres REQUIRED )
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   myMsg.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES parking_clean
  CATKIN_DEPENDS
        roscpp
        tf
        geometry_msgs
        rospy
        std_msgs
        client
        actionlib_msgs
        actionlib
#  DEPENDS
)

find_package(OpenCV 2.4.9 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src

  ${OpenCV_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}/opencv
  ${OpenCV_INCLUDE_DIRS}/opencv2
  /usr/include/eigen3
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/parkingclean.cpp
   src/laserdatadeal.cpp
   src/cpose2d.cpp
   src/datasave.cpp
   src/straightline.cpp
   src/line.cpp
#   src/quadtree.cpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(${PROJECT_NAME}_node src/parking_clean_node.cpp)


# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_parking_clean.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
target_link_libraries(${PROJECT_NAME}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
   ${catkin_LIBRARIES}
   pthread
   /usr/lib/x86_64-linux-gnu/libboost_system.so
   /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
   /usr/lib/x86_64-linux-gnu/libX11.so
)
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable( parkingClean_node src/parkingClean_node.cpp)
add_dependencies( parkingClean_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(parkingClean_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   pthread
   /usr/lib/x86_64-linux-gnu/libboost_system.so
   /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
   /usr/lib/x86_64-linux-gnu/libX11.so
)
