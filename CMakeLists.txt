cmake_minimum_required(VERSION 3.8)
project(schunk_control_egh80)
add_compile_options(-std=c++14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##################
## Dependencies ##
##################
set(ament_pkg_deps
  ament_cmake 
  rosidl_default_generators #To generate services in the package
  sensor_msgs
  std_msgs
  rclcpp
)
foreach(ament_pkg IN LISTS ament_pkg_deps)
    find_package(${ament_pkg} REQUIRED)
endforeach()
ament_export_dependencies(rosidl_default_runtime)

##################
#### includes ####
##################
# include_directories(
#   PUBLIC
#   include
# )
###########################
#### Generate Services ####
###########################

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ControlEGH80.srv"

  DEPENDENCIES std_msgs sensor_msgs
)

###########################
## Executables/Libraries ##
###########################

#########libraries#########

add_library(egh80_control
  SHARED 
  src/schunk_egh80_driver.cpp
)

ament_target_dependencies(egh80_control
  ${ament_pkg_deps})

target_include_directories(egh80_control
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rosidl_target_interfaces(egh80_control
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
) #to enable using services generated in this package

#########executables#########

add_executable(egh80_control_node
  src/schunk_egh80_driver_node.cpp
  # src/schunk_egh80_driver.cpp
)
target_link_libraries(egh80_control_node
  egh80_control) #required for locally compiled libraries

ament_target_dependencies(egh80_control_node
  ${ament_pkg_deps}) #only works with packages found via find_package

# target_include_directories(egh80_control_node PRIVATE 
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

#############
## Install ##
#############

#install includes
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

#install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)  

#libraries
install(
  TARGETS 
    egh80_control
    egh80_control_node
  # EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  # RUNTIME DESTINATION bin)
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
  # DESTINATION lib)
  # LIBRARY DESTINATION lib/${PROJECT_NAME}
  # ARCHIVE DESTINATION lib/${PROJECT_NAME}
  # RUNTIME DESTINATION lib/${PROJECT_NAME}
  # INCLUDES DESTINATION include

#targets
# install(
#   TARGETS
#     egh80_control
#     egh80_control_node
#   DESTINATION lib/${PROJECT_NAME})
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

#####################
## Declare package ##
####################
ament_export_dependencies(${ament_pkg_deps} rosidl_default_runtime)
ament_export_include_directories(include) #exporting header files for other packages to use it
ament_export_libraries(egh80_control  )
ament_package()
