cmake_minimum_required(VERSION 3.5)
project(pi_protocol_interface CXX C)
set(PROJECT_NAME ${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# pi-protocol checkout, cloned only when missing so an existing one is used
# untouched. The guard is a file, not the directory: an empty directory is a
# real state (uninitialised submodule, interrupted clone).
set(PI_PROTOCOL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pi-protocol_lib/pi-protocol)
if(NOT EXISTS ${PI_PROTOCOL_DIR}/config.yaml)
  message(STATUS "pi-protocol: cloning into ${PI_PROTOCOL_DIR}")
  include(FetchContent)
  fetchcontent_declare(
    pi_protocol_upstream
    GIT_REPOSITORY https://github.com/cvar-upm/pi-protocol.git
    GIT_TAG hexa_as2
    SOURCE_DIR ${PI_PROTOCOL_DIR}
  )
  fetchcontent_makeavailable(pi_protocol_upstream)
endif()

foreach(REQUIRED config.yaml msgs templates python/generate.py src/pi-protocol.c)
  if(NOT EXISTS ${PI_PROTOCOL_DIR}/${REQUIRED})
    message(FATAL_ERROR
      "pi-protocol: ${PI_PROTOCOL_DIR} has no ${REQUIRED}. Delete the directory to "
      "re-clone it, or point it at a complete pi-protocol checkout.")
  endif()
endforeach()

# Codec generated from the checkout's own message table, the one the firmware
# is built against. Needs python3 plus the checkout's python/requirements.txt.
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(PI_PROTOCOL_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${PI_PROTOCOL_GEN_DIR})
file(GLOB PI_PROTOCOL_MSGS ${PI_PROTOCOL_DIR}/msgs/*.yaml)
file(GLOB PI_PROTOCOL_TEMPLATES ${PI_PROTOCOL_DIR}/templates/*.j2)

add_custom_command(
  OUTPUT
    ${PI_PROTOCOL_GEN_DIR}/pi-protocol.h
    ${PI_PROTOCOL_GEN_DIR}/pi-messages.h
    ${PI_PROTOCOL_GEN_DIR}/pi-messages.c
  COMMAND ${Python3_EXECUTABLE} ${PI_PROTOCOL_DIR}/python/generate.py
          config.yaml --output-dir ${PI_PROTOCOL_GEN_DIR}
  WORKING_DIRECTORY ${PI_PROTOCOL_DIR}
  DEPENDS
    ${PI_PROTOCOL_DIR}/config.yaml
    ${PI_PROTOCOL_MSGS}
    ${PI_PROTOCOL_TEMPLATES}
    ${PI_PROTOCOL_DIR}/python/generate.py
  COMMENT "Generating pi-protocol codec"
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_generate DEPENDS ${PI_PROTOCOL_GEN_DIR}/pi-messages.c)

find_package(Threads REQUIRED)

# Only pi-messages.c is generated; src/pi-protocol.c ships in the checkout.
add_library(${PROJECT_NAME} STATIC
  ${PI_PROTOCOL_GEN_DIR}/pi-messages.c
  ${PI_PROTOCOL_DIR}/src/pi-protocol.c
  src/client.cpp
  src/clock_sync.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PI_PROTOCOL_GEN_DIR}
)
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)

# Plain CTest rather than ament_add_gtest, so this builds without ROS. The xml
# goes where colcon looks, to report individual cases instead of one per binary.
if(BUILD_TESTING)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    set(TEST_RESULTS_DIR ${CMAKE_BINARY_DIR}/test_results/${CMAKE_PROJECT_NAME})
    file(MAKE_DIRECTORY ${TEST_RESULTS_DIR})
    foreach(TEST_NAME parser clock_sync)
      set(TEST_TARGET ${PROJECT_NAME}_${TEST_NAME}_gtest)
      add_executable(${TEST_TARGET} tests/${TEST_NAME}_gtest.cpp)
      target_link_libraries(${TEST_TARGET} ${PROJECT_NAME} GTest::gtest_main)
      add_test(
        NAME ${TEST_TARGET}
        COMMAND ${TEST_TARGET}
                --gtest_output=xml:${TEST_RESULTS_DIR}/${TEST_TARGET}.gtest.xml)
    endforeach()
  else()
    message(STATUS "pi-protocol: GTest not found, skipping tests")
  endif()
endif()
