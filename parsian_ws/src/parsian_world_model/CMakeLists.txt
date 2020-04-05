cmake_minimum_required(VERSION 3.5)
project(parsian_world_model)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(parsian_msgs REQUIRED)
find_package(parsian_util REQUIRED)
find_package(Qt4 REQUIRED COMPONENTS QtCore)

# Qt4 configuration
include(${QT_USE_FILE})
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)
qt4_wrap_cpp(QT_MOC_HPP ${QT_MOC})
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)


# include paths
include_directories(
        include
        ${parsian_util_INCLUDE_DIRS}
)

# worldmodel node
add_executable(worldmodel
        src/worldmodel/worldmodel_exec.cpp
        src/worldmodel/worldmodel_node.cpp
        )
ament_target_dependencies(worldmodel rclcpp parsian_msgs parsian_util)
target_link_libraries(worldmodel ${QT_LIBRARIES})

# install nodes
install(TARGETS
        worldmodel
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()