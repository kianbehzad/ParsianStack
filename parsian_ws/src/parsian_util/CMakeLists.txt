cmake_minimum_required(VERSION 3.5)
project(parsian_util)

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
find_package(Qt4 REQUIRED COMPONENTS QtCore)

# Qt4 configuration
include(${QT_USE_FILE})
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)
qt4_wrap_cpp(QT_MOC_HPP ${QT_MOC})
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)


# include paths
include_directories(
        include
)


# create libknowledge library
#add_library(libknowledge
#        src/core/knowledge.cpp
#        )
#ament_target_dependencies(libknowledge rclcpp)
#ament_export_interfaces(export_libknowledge HAS_LIBRARY_TARGET)
#ament_export_libraries(libknowledge)
#ament_export_include_directories(include)
#ament_export_dependencies(
#        ament_cmake
#        rclcpp
#)

# create libgeom library
add_library(libgeom SHARED
        src/geom/angle_deg.cpp
#        src/geom/circle_2d.cpp
#        src/geom/geom.cpp
#        src/geom/line_2d.cpp
#        src/geom/matrix_2d.cpp
#        src/geom/polygon_2d.cpp
#        src/geom/ray_2d.cpp
#        src/geom/rect_2d.cpp
#        src/geom/sector_2d.cpp
#        src/geom/segment_2d.cpp
#        src/geom/triangle_2d.cpp
        src/geom/vector_2d.cpp
        )
ament_target_dependencies(libgeom rclcpp parsian_msgs)
ament_export_interfaces(export_libgeom HAS_LIBRARY_TARGET)
ament_export_libraries(libgeom)
ament_export_include_directories(include)
ament_export_dependencies(
        ament_cmake
        rclcpp
)

# create libmath library
#add_library(libmath
#        src/math/mathtools.cpp
#        src/math/matrix.cpp
#        )
#ament_target_dependencies(libmath rclcpp)
#target_link_libraries(libmath ${QT_LIBRARIES} libgeom)
#ament_export_interfaces(export_libmath HAS_LIBRARY_TARGET)
#ament_export_libraries(libmath)
#ament_export_include_directories(include)
#ament_export_dependencies(
#        ament_cmake
#        rclcpp
#)



# install include directory
install(
        DIRECTORY include/
        DESTINATION include
)



# install libknowledge
#install(
#        TARGETS libknowledge
#        EXPORT export_libknowledge
#        LIBRARY DESTINATION lib
#        ARCHIVE DESTINATION lib
#        RUNTIME DESTINATION bin
#        INCLUDES DESTINATION include
#)
# install libgeom
install(
        TARGETS libgeom
        EXPORT export_libgeom
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
# install libmath
#install(
#        TARGETS libmath
#        EXPORT export_libmath
#        LIBRARY DESTINATION lib
#        ARCHIVE DESTINATION lib
#        RUNTIME DESTINATION bin
#        INCLUDES DESTINATION include
#)



# install launch files
install(DIRECTORY tools/launch
  DESTINATION share/${PROJECT_NAME})

# install param files
install(DIRECTORY tools/params
  DESTINATION share/${PROJECT_NAME})


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
