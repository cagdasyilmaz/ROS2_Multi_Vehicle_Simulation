cmake_minimum_required(VERSION 3.5)
project(usv_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_COMPILER "/usr/bin/g++-11")
#set(CMAKE_CXX_COMPILER "/usr/bin/clang++-12")
#set(CMAKE_CXX_COMPILER "/opt/intel/oneapi/compiler/2023.1.0/linux/bin/icpx")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
    rclcpp
    geometry_msgs
    nav_msgs
)

add_executable(usv_01_controller src/usv_01_controller.cpp)
ament_target_dependencies(usv_01_controller ${dependencies})

add_executable(usv_02_controller src/usv_02_controller.cpp)
ament_target_dependencies(usv_02_controller ${dependencies})

add_executable(usv_03_controller src/usv_03_controller.cpp)
ament_target_dependencies(usv_03_controller ${dependencies})

add_executable(usv_04_controller src/usv_04_controller.cpp)
ament_target_dependencies(usv_04_controller ${dependencies})

add_executable(usv_05_controller src/usv_05_controller.cpp)
ament_target_dependencies(usv_05_controller ${dependencies})

add_executable(usv_06_controller src/usv_06_controller.cpp)
ament_target_dependencies(usv_06_controller ${dependencies})

install(TARGETS
  usv_01_controller
  usv_02_controller
  usv_03_controller
  usv_04_controller
  usv_05_controller
  usv_06_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
