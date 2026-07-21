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

# pi-protocol dependency (indiflight's high-rate telemetry channel, separate from MSP)
# lib/main/pi-protocol has no CMakeLists.txt of its own (it's YAML/Jinja2 templates + a
# generator script, not a CMake project), so it's populated (not add_subdirectory'd) and
# its C sources are generated/compiled explicitly below.
set(LOCAL_PI_PROTOCOL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pi-protocol)
if(NOT EXISTS ${LOCAL_PI_PROTOCOL_SRC}/config.yaml)
  message(STATUS "pi-protocol: cloning into ${LOCAL_PI_PROTOCOL_SRC}")
  include(FetchContent)
  fetchcontent_declare(
    pi_protocol_src
    GIT_REPOSITORY https://github.com/fjanguita/pi-protocol.git
    GIT_TAG 8716ac035fb54bbfdbb02f85c329f4845426f882  # motor-telemetry-support: adds MOTOR message
    SOURCE_DIR ${LOCAL_PI_PROTOCOL_SRC}
  )
  fetchcontent_populate(pi_protocol_src)
endif()

# Generating pi-protocol.h/pi-messages.{h,c} requires python3 + Jinja2/PyYAML/semver:
#   pip install -r thirdparty/pi-protocol/python/requirements.txt
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(PI_PROTOCOL_GEN_DIR ${CMAKE_BINARY_DIR}/pi_protocol_gen)
file(MAKE_DIRECTORY ${PI_PROTOCOL_GEN_DIR})

file(GLOB PI_PROTOCOL_MSG_YAML ${LOCAL_PI_PROTOCOL_SRC}/msgs/*.yaml)
file(GLOB PI_PROTOCOL_TEMPLATES ${LOCAL_PI_PROTOCOL_SRC}/templates/*.j2)

add_custom_command(
  OUTPUT
    ${PI_PROTOCOL_GEN_DIR}/pi-protocol.h
    ${PI_PROTOCOL_GEN_DIR}/pi-messages.h
    ${PI_PROTOCOL_GEN_DIR}/pi-messages.c
  COMMAND ${Python3_EXECUTABLE} ${LOCAL_PI_PROTOCOL_SRC}/python/generate.py
          config.yaml --output-dir ${PI_PROTOCOL_GEN_DIR}
  WORKING_DIRECTORY ${LOCAL_PI_PROTOCOL_SRC}
  DEPENDS
    ${LOCAL_PI_PROTOCOL_SRC}/config.yaml
    ${PI_PROTOCOL_MSG_YAML}
    ${PI_PROTOCOL_TEMPLATES}
    ${LOCAL_PI_PROTOCOL_SRC}/python/generate.py
  COMMENT "Generating pi-protocol C sources from YAML/Jinja2 templates"
  VERBATIM
)

add_custom_target(pi_protocol_generate
  DEPENDS
    ${PI_PROTOCOL_GEN_DIR}/pi-protocol.h
    ${PI_PROTOCOL_GEN_DIR}/pi-messages.h
    ${PI_PROTOCOL_GEN_DIR}/pi-messages.c
)

# src/pi-protocol.c is hand-written (not generated) and ships in the repo; only
# pi-messages.c is generated. Both #include the generated headers by name.
add_library(pi_protocol_gen STATIC
  ${PI_PROTOCOL_GEN_DIR}/pi-messages.c
  ${LOCAL_PI_PROTOCOL_SRC}/src/pi-protocol.c
)
add_dependencies(pi_protocol_gen pi_protocol_generate)
target_include_directories(pi_protocol_gen PUBLIC ${PI_PROTOCOL_GEN_DIR})

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
  src/pi_protocol_client.cpp
  src/pi_protocol_clock_sync.cpp
)

# Create the node executable
add_executable(${PROJECT_NAME}_node
  src/betaflight_platform_node.cpp
  ${SOURCE_CPP_FILES})

target_link_libraries(${PROJECT_NAME}_node
  msp_fcu
  pi_protocol_gen
)

ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

# # Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} msp_fcu pi_protocol_gen)

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
