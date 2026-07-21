# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.5)

# Set the project name
set(PROJECT_NAME as2_platform_indiflight)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default build type if not set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# MSP dependency
set(LOCAL_MSP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/msp)
find_package(msp QUIET)
if(${msp_FOUND})
  message(STATUS "msp found system-wide")
else()
  if(EXISTS ${LOCAL_MSP_SRC}/CMakeLists.txt)
    message(STATUS "msp: using local clone at ${LOCAL_MSP_SRC}")
    add_subdirectory(${LOCAL_MSP_SRC} ${CMAKE_BINARY_DIR}/msp_build)
  else()
    message(STATUS "msp: cloning into ${LOCAL_MSP_SRC}")
    include(FetchContent)
    fetchcontent_declare(
      msp
      GIT_REPOSITORY https://github.com/CVAR-A2RL/msp.git
      GIT_TAG master
      SOURCE_DIR ${LOCAL_MSP_SRC}
    )
    fetchcontent_makeavailable(msp)
  endif()
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  as2_msgs
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include necessary directories
include_directories(
  include
  include/${PROJECT_NAME}
  ${LOCAL_MSP_SRC}/inc
)

# Set source files
set(SOURCE_CPP_FILES
  src/betaflight_platform.cpp
)

# Create the node executable
add_executable(${PROJECT_NAME}_node
  src/betaflight_platform_node.cpp
  ${SOURCE_CPP_FILES})

target_link_libraries(${PROJECT_NAME}_node
  msp_fcu
)

ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

# # Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} msp_fcu)

# Install the headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the node executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# # Install the launch directory
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Install the config directory
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

# Build tests if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Keep all ament linting tools away from thirdparty/ (fetched at configure time,
  # gitignored). All ament crawlers skip any directory that contains AMENT_IGNORE.
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
  file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/AMENT_IGNORE "")

  ament_lint_auto_find_test_dependencies()

  add_subdirectory(tests)
endif()

# Create the ament package
ament_package()
