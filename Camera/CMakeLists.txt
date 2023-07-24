cmake_minimum_required(VERSION 3.0.2)
project(Camera)

set(CMAKE_BUILD_TYPE RELEASE)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  mavros_msgs
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)

find_package(OpenCV 3.4.1 REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
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
#   geometry_msgs#   mavros_msgs#   sensor_msgs#   tf2_geometry_msgs#   tf2_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Camera
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transprot mavros_msgs roscpp sensor_msgs tf2 tf2_geometry_msgs tf2_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/Camera.cpp
# )

add_library(AAMED_LIBRARIES
  include/AAMED/adaptApproximateContours.h
  include/AAMED/definition.h
  include/AAMED/EllipseConstraint.h
  include/AAMED/EllipseNonMaximumSuppression.h
  include/AAMED/FLED.h
  include/AAMED/LinkMatrix.h
  include/AAMED/Node_FC.h
  include/AAMED/src/adaptApproximateContours.cpp
  include/AAMED/src/adaptApproxPolyDP.cpp
  include/AAMED/src/Contours.cpp
  include/AAMED/src/EllipseNonMaximumSuppression.cpp
  include/AAMED/src/FLED_drawAndWriteFunctions.cpp
  include/AAMED/src/FLED_Initialization.cpp
  include/AAMED/src/FLED_PrivateFunctions.cpp
  include/AAMED/src/FLED.cpp
  include/AAMED/src/Group.cpp
  include/AAMED/src/LinkMatrix.cpp
  include/AAMED/src/Node_FC.cpp
  include/AAMED/src/Segmentation.cpp
  include/AAMED/src/Validation.cpp
)

add_library(DBSCAN_LIBRARIES
  include/DBSCAN/dbscan.h
  include/DBSCAN/dbscan.cpp
)

add_library(ECO_LIBRARIES
  include/eco/include/debug.hpp
  include/eco/include/eco.hpp
  include/eco/include/feature_extractor.hpp
  include/eco/include/feature_operator.hpp
  include/eco/include/ffttools.hpp
  include/eco/include/fhog.hpp
  include/eco/include/gradient.hpp
  include/eco/include/interpolator.hpp
  include/eco/include/metrics.hpp
  include/eco/include/optimize_scores.hpp
  include/eco/include/parameters.hpp
  include/eco/include/recttools.hpp
  include/eco/include/regularization_filter.hpp
  include/eco/include/sample_update.hpp
  include/eco/include/scale_filter.hpp
  include/eco/include/sse.hpp
  include/eco/include/sse2neon.h
  include/eco/include/training.hpp
  include/eco/include/vot.h
  include/eco/include/wrappers.hpp
  include/eco/src/eco.cc
  include/eco/src/feature_extractor.cc
  include/eco/src/feature_operator.cc
  include/eco/src/ffttools.cc
  include/eco/src/fhog.cc
  include/eco/src/gradient.cpp
  include/eco/src/interpolator.cc
  include/eco/src/metrics.cc
  include/eco/src/optimize_scores.cc
  include/eco/src/regularization_filter.cc
  include/eco/src/sample_update.cc
  include/eco/src/scale_filter.cc
  include/eco/src/training.cc
)

add_library(KCF_LIBRARIES
  include/KCF/include/ffttools.hpp
  include/KCF/include/fhog.hpp
  include/KCF/include/kcftracker.hpp
  include/KCF/include/labdata.hpp
  include/KCF/include/recttools.hpp
  include/KCF/src/fhog.cpp
  include/KCF/src/kcftracker.cpp
)

add_library(CAMERA_LIBARAIES
  include/Camera/camera.h
  include/Camera/camera.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(AAMED_LIBRARIES ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(DBSCAN_LIBRARIES ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ECO_LIBRARIES ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(KCF_LIBRARIES ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(CAMERA_LIBARAIES ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(CAMERA_LIBARAIES
  KCF_LIBRARIES
  ECO_LIBRARIES
  AAMED_LIBRARIES
  DBSCAN_LIBRARIES
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/Camera_node.cpp)
add_executable(main src/main.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(main
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  CAMERA_LIBARAIES
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_Camera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
