cmake_minimum_required(VERSION 3.5.0)
project(tesseract_planning_nodes VERSION 0.1.0 LANGUAGES CXX)

find_package(tesseract REQUIRED)
find_package(Eigen3 REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_17 CXX_FEATURE_FOUND)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(tesseract_rosutils REQUIRED)
find_package(tesseract_motion_planners REQUIRED)

# Tesseract ROS Nodes
add_executable(${PROJECT_NAME}_planning_worker_node src/planning_worker_node.cpp)
target_link_libraries(${PROJECT_NAME}_planning_worker_node PRIVATE
    tesseract::tesseract
    tesseract::tesseract_rosutils
    tesseract::tesseract_motion_planners_ompl
    ${rclcpp_LIBRARIES}
    ${rclcpp_action_LIBRARIES}
    ${tesseract_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    )
target_compile_options(${PROJECT_NAME}_planning_worker_node PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
set_target_properties(${PROJECT_NAME}_planning_worker_node PROPERTIES CXX_STANDARD 14)
target_include_directories(${PROJECT_NAME}_planning_worker_node PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_planning_worker_node SYSTEM PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_action_INCLUDE_DIRS}
    ${tesseract_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_planning_worker_node)

ament_export_dependencies(
    roscpp
    tesseract_msgs
    tesseract_rosutils
    tesseract_motion_planners
    tf2_ros
    tf2_eigen
    EIGEN3
    tesseract
    )

ament_export_include_directories(include)

ament_package()

install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: DESTINATION lib/cmake/${PROJECT_NAME})

install(TARGETS
   ${PROJECT_NAME}_planning_worker_node
   DESTINATION lib/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
