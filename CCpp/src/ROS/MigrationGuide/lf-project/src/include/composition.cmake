find_package(ament_cmake REQUIRED)
find_package(lf_simple REQUIRED)
ament_target_dependencies(${LF_MAIN_TARGET} lf_simple)
