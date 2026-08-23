cmake_minimum_required(VERSION 3.20)

# PCL 的 FindFLANN.cmake 会触发 CMP0144 开发警告，显式采用新策略消除
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# 必须启用 C 语言：Ubuntu 24.04 的 VTK::mpi 链接接口引用 MPI::MPI_C（C 绑定），
# 纯 CXX 工程里 find_package(MPI) 只定义 MPI::MPI_CXX，会导致 VTK 报 target not found
project(datacache LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)
find_path(ZSTD_INCLUDE_DIR NAMES zstd.h REQUIRED)
find_library(ZSTD_LIBRARY NAMES zstd REQUIRED)
find_package(CURL REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EventTrigger.srv"
)

# 必须在 pcl_conversions/PCL 之前查找，且显式要求 C、CXX 两种绑定
find_package(MPI REQUIRED COMPONENTS C CXX)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# 5 个可执行文件, 目标名与 launch 文件里的 executable 一致(record_reader 为 CLI 工具)
add_executable(datacache_node src/main.cc src/datacache_node.cpp src/config_manager.cpp)
add_executable(camera_node src/camera_node.cpp)
add_executable(lidar_sim_node src/lidar_sim_node.cc)
add_executable(event_trigger_node src/event_trigger_node.cc)
add_executable(record_reader src/record_reader.cpp)

foreach(target datacache_node camera_node lidar_sim_node event_trigger_node record_reader)
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(${target}
    rclcpp sensor_msgs)
endforeach()

rosidl_target_interfaces(datacache_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(event_trigger_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 各节点额外的库依赖
target_link_libraries(camera_node ${OpenCV_LIBS})
target_link_libraries(camera_node cv_bridge::cv_bridge)
target_link_libraries(datacache_node Threads::Threads)
target_include_directories(datacache_node PRIVATE ${ZSTD_INCLUDE_DIR})
target_link_libraries(datacache_node ${ZSTD_LIBRARY})
target_link_libraries(datacache_node CURL::libcurl)
target_link_libraries(datacache_node ${OpenCV_LIBS})
target_include_directories(datacache_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(datacache_node ${PCL_LIBRARIES})
ament_target_dependencies(datacache_node pcl_conversions)

# record_reader: zstd 解压 + OpenCV/PCL 导出
target_include_directories(record_reader PRIVATE ${ZSTD_INCLUDE_DIR})
target_link_libraries(record_reader ${ZSTD_LIBRARY} ${OpenCV_LIBS})
target_include_directories(record_reader PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(record_reader ${PCL_LIBRARIES})
ament_target_dependencies(record_reader pcl_conversions)

# pcl_conversions 必须走 ament_target_dependencies 才能带出头文件路径
ament_target_dependencies(lidar_sim_node pcl_conversions)
target_include_directories(lidar_sim_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar_sim_node ${PCL_LIBRARIES})

install(TARGETS datacache_node camera_node lidar_sim_node event_trigger_node record_reader
  DESTINATION lib/${PROJECT_NAME})

# launch / config / pcd 装进 install 空间，运行时才能找到
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES config.txt DESTINATION share/${PROJECT_NAME})
install(DIRECTORY pcd DESTINATION share/${PROJECT_NAME})
install(FILES tools/mock_server.py DESTINATION share/${PROJECT_NAME}/tools
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE)


ament_package()
