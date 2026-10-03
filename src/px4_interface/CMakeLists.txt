cmake_minimum_required(VERSION 3.14) # 3.14+ 支持更丰富的 target_* 功能与生成表达式改进

# ============================= 项目信息与全局策略 =============================
project(px4_interface
  VERSION 0.1.0
  DESCRIPTION "PX4 相关的中间层接口（缓存 + 网关），同时包含本包自定义 msg"
  LANGUAGES CXX C
)

# 避免使用全局 add_compile_options 影响所有 target；
# 更推荐在每个 target 上用 target_compile_options 精准设置。
# 这里保留一个最低限度的全局策略：启用严格警告（仅对 GNU/Clang）。
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 为了鼓励使用更现代的 C++ 标准（并保持编译一致性），强制 C++17。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================= 依赖查找 =============================
# ament_cmake 是 ROS 2 的基础构建系统封装。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)          # 头文件库（INTERFACE 库），不会产生链接库
find_package(px4_msgs REQUIRED)        # 来自 px4_msgs 包的已生成消息接口
find_package(nav_msgs REQUIRED)        # nav_msgs/Odometry for VSLAM bridge
find_package(rosidl_default_generators REQUIRED) # 生成本包自定义消息所需

# ============================= 自定义消息生成 =============================
# 如果需要在别的包使用这些 msg，需要在 package.xml 中声明依赖
# 并且保证 rosidl_generate_interfaces 在 ament_package() 之前调用。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleStatus.msg"
  "msg/PoseNED.msg"
  "msg/BatteryStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# 生成的 C++ 头会放在构建目录（build/ 本包）内，对应生成路径已经被 rosidl 系统管理。
# 旧式写法 include_directories(...) 会污染全局作用域，改为对 target 指定。

# ============================= 公共头文件基路径变量 =============================
set(PROJECT_PUBLIC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

#============================== process_manager 库 =============================
add_library(process_manager
  src/process_manager.cpp
)
target_include_directories(process_manager
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_PUBLIC_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# 这里不需要额外依赖
target_compile_options(process_manager
  PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wconversion -Wshadow>
)

# ============================= px4_msgs_cache 库 =============================
add_library(px4_msgs_cache
  src/px4_msgs_cache.cpp
)

# 指定此库的“使用接口”包含目录：
target_include_directories(px4_msgs_cache
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_PUBLIC_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# 链接依赖：使用 ament_target_dependencies 让 ROS 2 自动追加所需 include 与库。
ament_target_dependencies(px4_msgs_cache
  PUBLIC  # 这些依赖对使用者可见（例如使用本库的其他目标会自动继承）
    rclcpp
    std_msgs
    Eigen3 # ament 会追加包含；某些平台需显式 target_link_libraries(Eigen3::Eigen)
    rosidl_default_runtime # 使用已生成的消息运行时支持
)

# 某些发行版的 Eigen3 包只通过 INTERFACE 目标 Eigen3::Eigen 提供包含路径，这里显式链接一次
target_link_libraries(px4_msgs_cache PUBLIC Eigen3::Eigen)

# 额外编译选项（只作用于该库）
target_compile_options(px4_msgs_cache
  PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wconversion -Wshadow>
)

# ============================= px4_gateway 库 =============================
add_library(px4_gateway
  src/px4_gateway.cpp
)

target_include_directories(px4_gateway
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_PUBLIC_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# 这里依赖前一个自定义库 px4_msgs_cache 以及额外消息包 px4_msgs。
target_link_libraries(px4_gateway
  PUBLIC
    px4_msgs_cache
    process_manager
    Eigen3::Eigen # 显式使用 Eigen 导出的接口目标，确保包含目录存在
)

ament_target_dependencies(px4_gateway
  PUBLIC
    rclcpp
    std_msgs
    Eigen3
    px4_msgs           # 来自外部包的消息类型
    rosidl_default_runtime
)

# 确保在编译 px4_gateway 之前消息代码已生成（__cpp 目标由 rosidl 生成）
add_dependencies(px4_gateway ${PROJECT_NAME}__cpp)

# 如需为不同 target 设置定义，可使用 target_compile_definitions：
# target_compile_definitions(px4_gateway PRIVATE SOME_FLAG=1)

# ============================= px4_gateway_node 可执行文件 =============================
add_executable(px4_gateway_node src/main.cpp)

target_link_libraries(px4_gateway_node px4_gateway)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(px4_gateway_node ${cpp_typesupport_target})

target_include_directories(px4_gateway_node PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

add_dependencies(px4_gateway_node ${PROJECT_NAME}__cpp)

# ============================= vslam_odom_bridge_node =============================
add_executable(vslam_odom_bridge_node src/vslam_odom_bridge_main.cpp
                                        src/vslam_odom_bridge.cpp)

target_include_directories(vslam_odom_bridge_node
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_PUBLIC_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(vslam_odom_bridge_node
  PRIVATE
    Eigen3::Eigen
)

ament_target_dependencies(vslam_odom_bridge_node
  PUBLIC
    rclcpp
    px4_msgs
    nav_msgs
    rosidl_default_runtime
)

# ============================= 安装 (Install) =============================
# 安装库到 lib；安装头文件到 include；
# 根据 ROS 2 约定放在 lib/${PROJECT_NAME} 不是必须——对纯库包可直接放 lib。
install(TARGETS
  process_manager
  px4_msgs_cache
  px4_gateway
  px4_gateway_node
  vslam_odom_bridge_node
  EXPORT ${PROJECT_NAME}-targets           # 导出安装时的 target 描述，便于外部 find_package
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY ${PROJECT_PUBLIC_INCLUDE_DIR}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# 安装脚本
# 注: lidar_tf_relay.py / scan_planar.py / laser_icp_odom.py（激光 SLAM）与
#     openvins_odom_relay.py（OpenVINS）已于 2026-09-14 随激光/T265/OpenVINS 清理移除，
#     RPLidar 仅保留避障链路（uav_task sensors_extra.launch.py）。
install(
  PROGRAMS
    scripts/mock_px4_cache_publisher.py
    scripts/state_bridge.py
    scripts/rtabmap_odom_relay.py
    scripts/px4_imu_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Launch 文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================= 测试 (可选) =============================
if(BUILD_TESTING)
  # ament_lint_auto: 自动化静态检查集合（风格/版权等）
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # 设置为 TRUE 用于跳过某些检查
  set(ament_cmake_uncrustify_FOUND TRUE) # 使用clang-format，不使用 uncrustify
  set(ament_cmake_cpplint_FOUND TRUE) # 使用 clang-format，不使用 cpplint
  set(ament_cmake_copyright_FOUND TRUE) # 版权在 package.xml 中声明

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_px4_msgs_cache test/test_px4_msgs_cache.cpp)
  if(TARGET test_px4_msgs_cache)
    target_link_libraries(test_px4_msgs_cache px4_msgs_cache)
    target_include_directories(test_px4_msgs_cache PRIVATE ${PROJECT_PUBLIC_INCLUDE_DIR})
    ament_target_dependencies(test_px4_msgs_cache rclcpp std_msgs Eigen3)
  endif()

  ament_add_gtest(test_px4_gateway test/test_px4_gateway.cpp)
  if(TARGET test_px4_gateway)
    target_link_libraries(test_px4_gateway px4_gateway px4_msgs_cache Eigen3::Eigen)
    target_include_directories(test_px4_gateway PRIVATE
      ${PROJECT_PUBLIC_INCLUDE_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
    )
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(test_px4_gateway ${cpp_typesupport_target})
    ament_target_dependencies(test_px4_gateway rclcpp std_msgs Eigen3 px4_msgs rosidl_default_runtime)
    add_dependencies(test_px4_gateway ${PROJECT_NAME}__cpp)
  endif()

  ament_add_gtest(test_process_manager test/test_process_manager.cpp)
  if(TARGET test_process_manager)
    target_link_libraries(test_process_manager process_manager)
    target_include_directories(test_process_manager PRIVATE ${PROJECT_PUBLIC_INCLUDE_DIR})
  endif()

  ament_add_gtest(test_vslam_odom_math test/test_vslam_odom_math.cpp)
  if(TARGET test_vslam_odom_math)
    target_include_directories(test_vslam_odom_math PRIVATE ${PROJECT_PUBLIC_INCLUDE_DIR})
    target_link_libraries(test_vslam_odom_math Eigen3::Eigen)
    ament_target_dependencies(test_vslam_odom_math Eigen3)
  endif()
endif()

# ============================= 包导出 =============================
# ament_export_targets 导出本包的 CMake target，供下游包通过 find_package 使用。
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

# 导出依赖，确保 find_package(px4_interface) 时自动带上包含路径/库与依赖的顺序。
ament_export_dependencies(
  rclcpp
  std_msgs
  Eigen3
  px4_msgs
  rosidl_default_runtime
)

# ament 包尾：必须在 rosidl_generate_interfaces 之后。
ament_package()


# ============================= 新手快速 FAQ =============================
# 1. 为什么不用 include_directories()?  -> 它是全局的，会影响所有 target；现代写法更推荐 target_include_directories 精准控制作用域。
# 2. PUBLIC / PRIVATE / INTERFACE 有何区别? -> 传播范围：PUBLIC=自己+依赖者；PRIVATE=仅自己；INTERFACE=自己没有源或只传播给依赖者。
# 3. ament_target_dependencies 做了什么? -> 相当于 target_link_libraries + 自动追加 include 路径 + 添加系统依赖，减少手写。
# 4. 如何新增一个源文件? -> 把文件追加到 add_library 的列表，或使用 target_sources。
# 5. 如何添加可执行节点?
#    add_executable(node_name src/node.cpp);
#    然后 ament_target_dependencies(node_name rclcpp ...)
#    并 install(TARGETS node_name ... RUNTIME DESTINATION lib/${PROJECT_NAME}).
# 6. 为什么导出 targets? -> 便于下游使用 find_package(px4_interface) 时拿到配置 & 链接信息，支持更好的复用。
