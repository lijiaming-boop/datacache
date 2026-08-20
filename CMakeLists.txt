cmake_minimum_required(VERSION 3.20)

# PCL 的 FindFLANN.cmake 会触发 CMP0144 开发警告，显式采用新策略消除
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

project(datacache LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 必须在 pcl_conversions/PCL 之前查找：VTK 的 VTK::mpi 目标链接 MPI::MPI_C，
# 系统 MPI 开发库缺失时 VTK-targets.cmake 会直接报 "target was not found"
find_package(MPI REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# 4 个可执行文件，目标名与 launch 文件里的 executable 一致
add_executable(datacache_node src/main.cc)
add_executable(camera_node src/camera_node.cpp)
add_executable(lidar_sim_node src/lidar_sim_node.cc)
add_executable(event_trigger_node src/event_trigger_node.cc)

foreach(target datacache_node camera_node lidar_sim_node event_trigger_node)
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(${target}
    rclcpp sensor_msgs std_srvs)
endforeach()

# 各节点额外的库依赖
target_link_libraries(camera_node ${OpenCV_LIBS})
target_link_libraries(camera_node cv_bridge::cv_bridge)
target_link_libraries(lidar_sim_node ${PCL_LIBRARIES})
target_link_libraries(lidar_sim_node pcl_conversions)

install(TARGETS datacache_node camera_node lidar_sim_node event_trigger_node
  DESTINATION lib/${PROJECT_NAME})

# launch / config / pcd 装进 install 空间，运行时才能找到
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES config.txt DESTINATION share/${PROJECT_NAME})
install(DIRECTORY pcd DESTINATION share/${PROJECT_NAME})

ament_package()
