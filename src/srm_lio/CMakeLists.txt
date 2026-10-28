cmake_minimum_required(VERSION 3.16)
project(srm_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 QUIET)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(OpenMP QUIET)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(yaml-cpp REQUIRED)


set(SRC_LIST src/map_builder/commons.cpp
             src/map_builder/ieskf.cpp
             src/map_builder/imu_processor.cpp
             src/map_builder/ikd_Tree.cpp
             src/map_builder/lidar_processor.cpp
             src/map_builder/structure_reliability.cpp
             src/map_builder/map_builder.cpp
             src/utils.cpp)

add_executable(lio_node src/lio_node.cpp ${SRC_LIST})
if(OpenMP_CXX_FOUND)
  target_compile_definitions(lio_node PRIVATE MP_EN MP_PROC_NUM=2)
  target_link_libraries(lio_node OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(lio_node rclcpp std_msgs tf2 tf2_ros nav_msgs sensor_msgs pcl_conversions geometry_msgs)
if(livox_ros_driver2_FOUND)
  target_compile_definitions(lio_node PRIVATE SRM_HAS_LIVOX=1)
  ament_target_dependencies(lio_node livox_ros_driver2)
else()
  target_compile_definitions(lio_node PRIVATE SRM_HAS_LIVOX=0)
  message(WARNING "livox_ros_driver2 not found: building PointCloud2 input support only")
endif()
target_link_libraries(lio_node
  yaml-cpp
  Eigen3::Eigen
  Sophus::Sophus
  ${PCL_LIBRARIES}
)


install(TARGETS lio_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_executable(test_structure_reliability test/test_structure_reliability.cpp
                 src/map_builder/structure_reliability.cpp src/map_builder/commons.cpp)
  target_include_directories(test_structure_reliability PRIVATE src/map_builder ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_structure_reliability ${PCL_LIBRARIES} Eigen3::Eigen Sophus::Sophus)
  add_test(NAME structure_reliability COMMAND test_structure_reliability)
endif()

ament_package()
