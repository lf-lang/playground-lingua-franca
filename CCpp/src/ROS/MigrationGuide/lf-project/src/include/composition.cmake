find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lf_simple REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} rclcpp)
ament_target_dependencies(${LF_MAIN_TARGET} lf_simple)
ament_target_dependencies(${LF_MAIN_TARGET} std_msgs)

include_directories("../../../../lf_simple_package/lf_simple/src")