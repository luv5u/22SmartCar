cmake_minimum_required(VERSION 3.8)
project(m3_navigation)
find_package(ament_cmake REQUIRED)
configure_file(ament/bringup.launch.py.in generated/bringup.launch.py @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/bringup.launch.py"
        DESTINATION share/${PROJECT_NAME}/launch)
ament_package()
