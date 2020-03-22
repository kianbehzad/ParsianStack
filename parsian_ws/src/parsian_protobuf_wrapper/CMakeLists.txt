cmake_minimum_required(VERSION 3.5)
project(parsian_protobuf_wrapper)

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
find_package(Qt4 REQUIRED COMPONENTS QtNetwork QtCore)
find_package(Protobuf REQUIRED)

# Qt4 configuration
include(${QT_USE_FILE})
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)
qt4_wrap_cpp(QT_MOC_HPP ${QT_MOC})
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

# compile proto files
set(proto_dir ${PROJECT_SOURCE_DIR}/proto/)
file(GLOB PROTOBUF_DEFINITION_FILES "${proto_dir}/*.proto")
set(PROTOBUF_INPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/proto")
set(PROTOBUF_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/include/parsian_protobuf_wrapper/proto")
foreach(file ${PROTOBUF_DEFINITION_FILES})
    set(PROTOBUF_COMMAND protoc ${file})
    set(PROTOBUF_ARGUMENTS --proto_path=${PROTOBUF_INPUT_DIRECTORY} --cpp_out=${PROTOBUF_OUTPUT_DIRECTORY})
    execute_process(COMMAND ${PROTOBUF_COMMAND} ${PROTOBUF_ARGUMENTS}
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            RESULT_VARIABLE PROTOBUF_RESULT
            OUTPUT_VARIABLE PROTOBUF_OUTPUT_VARIABLE
            ERROR_VARIABLE PROTOBUF_ERROR)
endforeach()


# include paths
include_directories(
        include
        ${parsian_util_INCLUDE_DIRS}
)

# grsim node
add_executable(grsim
        src/grsim/grsim_exec.cpp
        src/grsim/grsim_node.cpp
        src/common/net/udpsend.cpp
        include/parsian_protobuf_wrapper/proto/grSim_Packet.pb.cc
        include/parsian_protobuf_wrapper/proto/grSim_Commands.pb.cc
        include/parsian_protobuf_wrapper/proto/grSim_Replacement.pb.cc
        )
ament_target_dependencies(grsim rclcpp parsian_msgs parsian_util)
target_link_libraries(grsim ${QT_LIBRARIES} ${PROTOBUF_LIBRARIES})


# vision node
add_executable(vision
        src/vision/vision_exec.cpp
        src/vision/vision_node.cpp
        )
ament_target_dependencies(vision rclcpp parsian_msgs parsian_util)
target_link_libraries(vision ${QT_LIBRARIES} ${PROTOBUF_LIBRARIES})



# install nodes
install(TARGETS
  grsim
  vision
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
