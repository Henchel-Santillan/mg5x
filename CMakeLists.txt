cmake_minimum_required(VERSION 3.21)
project(mg5x)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(TESTING "Specifies whether the test String message and image capture logic should be used" OFF)

add_subdirectory(extern/fmt)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(MG5X_EXECUTABLE mg5x-main)
list(APPEND MG5X_SOURCES
      "src/image_capture_node.cpp"
      "src/main.cpp"
      "src/session.cpp")

add_executable(${MG5X_EXECUTABLE} ${MG5X_SOURCES})
ament_target_dependencies(${MG5X_EXECUTABLE} rclcpp px4_msgs std_msgs)

target_include_directories(${MG5X_EXECUTABLE} PRIVATE 
  "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}"
  "${CMAKE_CURRENT_SOURCE_DIR}/extern/fmt/include")

target_link_libraries(${MG5X_EXECUTABLE} fmt::fmt)
target_compile_features(${MG5X_EXECUTABLE} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

if(TESTING)
  target_compile_definitions(${MG5X_EXECUTABLE} PRIVATE CAPTURE_TESTING)
endif()

install(TARGETS ${MG5X_EXECUTABLE}
  DESTINATION lib/${PROJECT_NAME})

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
endif()

ament_package()
