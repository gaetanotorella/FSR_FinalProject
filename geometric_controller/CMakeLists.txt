cmake_minimum_required(VERSION 2.8.3)
project(geometric_controller)

add_definitions(-std=c++14)

#--------------------------#
# Generation of SDF models #
#--------------------------#

# We need Gazebo version >= 3.0.0 to generate iris.sdf file
# (gz sdf ... command needs to be available)
find_package(gazebo REQUIRED)

# if(${gazebo_VERSION_MAJOR} GREATER 2)
#     message(STATUS "Building iris.sdf.")
    
#     set(enable_mavlink_interface "true")
#     set(enable_ground_truth "false")
#     set(enable_logging "false")
#     set(enable_camera "false")
#     set(enable_wind "false")
#     set(rotors_description_dir "${CMAKE_CURRENT_SOURCE_DIR}")
#     set(scripts_dir "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
    
#     # Creates shell commands to generate .sdf file
#     add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
#       WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#       COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
#       COMMAND python3 ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/iris_base.urdf  ${rotors_description_dir}/urdf/iris_base.xacro enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_wind:=${enable_wind} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir}
#       COMMAND gz sdf -p  ${rotors_description_dir}/urdf/iris_base.urdf >> ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
#       COMMAND rm -f ${rotors_description_dir}/urdf/iris_base.urdf
#       DEPENDS ${rotors_description_dir}/urdf/iris.xacro
#       DEPENDS ${rotors_description_dir}/urdf/iris_base.xacro
#       DEPENDS ${rotors_description_dir}/urdf/component_snippets.xacro
#       )
#     add_custom_target(sdf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf)
# else()
#   message(STATUS "Gazebo version is less than 3, cannot generate iris.sdf (gz tool not available).")
# endif()

if(NO_ROS)
  message(STATUS "NO_ROS = true, not building waypoint publisher or hovering example, returning now.")
  return()
endif()

find_package(catkin REQUIRED COMPONENTS gazebo_msgs geometry_msgs mav_msgs roscpp sensor_msgs laser_geometry tf message_filters)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    mav_msgs
    roscpp
    sensor_msgs
    laser_geometry
    tf
    message_filters
    ## INCLUDE NOSTRI
    INCLUDE_DIRS include
    LIBRARIES geometric_controller
)

## LIBRERIA C++ NOSTRA
## Declare a C++ library
add_library(geometric_controller src/utils_fun.cpp)


include_directories(include ${catkin_INCLUDE_DIRS})


## EXECUTABLE NOSTRI

add_executable(outer_loop src/outer_loop.cpp)
target_link_libraries(outer_loop geometric_controller ${catkin_LIBRARIES})
add_dependencies(outer_loop ${catkin_EXPORTED_TARGETS})

add_executable(inner_loop src/inner_loop.cpp)
target_link_libraries(inner_loop geometric_controller ${catkin_LIBRARIES})
add_dependencies(inner_loop ${catkin_EXPORTED_TARGETS})

add_executable(outer_loop_apf src/outer_loop_apf.cpp)
target_link_libraries(outer_loop_apf geometric_controller ${catkin_LIBRARIES})
add_dependencies(outer_loop_apf ${catkin_EXPORTED_TARGETS})

add_executable(APF_method_offline src/APF_method_offline.cpp)
target_link_libraries(APF_method_offline geometric_controller ${catkin_LIBRARIES})
add_dependencies(APF_method_offline ${catkin_EXPORTED_TARGETS})

add_executable(APF_method_online src/APF_method_online.cpp)
target_link_libraries(APF_method_online geometric_controller ${catkin_LIBRARIES})
add_dependencies(APF_method_online ${catkin_EXPORTED_TARGETS})



foreach(dir launch models resource worlds)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# install(TARGETS waypoint_publisher waypoint_publisher_file hovering_example
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
