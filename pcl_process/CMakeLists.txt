cmake_minimum_required(VERSION 3.5)
project(pcl_process)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} )
link_libraries(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)


add_executable(pc_listener src/subscriber_livox.cpp)
target_link_libraries(pc_listener ${PCL_LIBRARIES})
ament_target_dependencies(pc_listener rclcpp sensor_msgs pcl_conversions PCL)

add_executable(pc_boundary src/subscriber_boundary.cpp)
target_link_libraries(pc_boundary ${PCL_LIBRARIES})
ament_target_dependencies(pc_boundary rclcpp sensor_msgs pcl_conversions PCL)

add_executable(ransac src/subscriber_line.cpp)
target_link_libraries(ransac ${PCL_LIBRARIES})
ament_target_dependencies(ransac rclcpp sensor_msgs pcl_conversions PCL)

add_executable(wall_follower src/follow_wall.cpp)
target_link_libraries(wall_follower ${PCL_LIBRARIES})
ament_target_dependencies(wall_follower rclcpp sensor_msgs pcl_conversions PCL)

add_executable(regionGrowing src/region.cpp)
target_link_libraries(regionGrowing ${PCL_LIBRARIES})
ament_target_dependencies(regionGrowing rclcpp sensor_msgs pcl_conversions PCL)

add_executable(pcl_wall_follower src/pcl_wall_follower.cpp)
target_link_libraries(pcl_wall_follower ${PCL_LIBRARIES})
ament_target_dependencies(pcl_wall_follower rclcpp sensor_msgs pcl_conversions PCL)

add_executable(pcl_ransac_filter src/pcl_ransac_filter.cpp)
target_link_libraries(pcl_ransac_filter ${PCL_LIBRARIES})
ament_target_dependencies(pcl_ransac_filter rclcpp sensor_msgs pcl_conversions PCL)

add_executable(curve_ransac src/curve_ransac.cpp)
target_link_libraries(curve_ransac ${PCL_LIBRARIES})
ament_target_dependencies(curve_ransac rclcpp sensor_msgs pcl_conversions PCL)

add_executable(timer src/timer.cpp)
target_link_libraries(timer ${PCL_LIBRARIES})
ament_target_dependencies(timer rclcpp sensor_msgs pcl_conversions PCL)

add_executable(bend src/bend.cpp)
target_link_libraries(bend ${PCL_LIBRARIES})
ament_target_dependencies(bend rclcpp sensor_msgs pcl_conversions PCL)

add_executable(bag_to_pcd src/bag_to_pcd.cpp)
target_link_libraries(bag_to_pcd ${PCL_LIBRARIES})
ament_target_dependencies(bag_to_pcd rclcpp sensor_msgs pcl_conversions PCL)

add_executable(publish_cloud src/publish_cloud.cpp)
target_link_libraries(publish_cloud ${PCL_LIBRARIES})
ament_target_dependencies(publish_cloud rclcpp sensor_msgs pcl_conversions PCL)

install(TARGETS pc_listener pc_boundary ransac wall_follower regionGrowing pcl_wall_follower curve_ransac pcl_ransac_filter timer bend bag_to_pcd publish_cloud
  DESTINATION lib/${PROJECT_NAME})


ament_package()
