# GTest
file(GLOB GTEST_SOURCE "*_gtest.cpp")

if(GTEST_SOURCE)
  find_package(ament_cmake_gtest REQUIRED)

  foreach(TEST_SOURCE ${GTEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

    ament_add_gtest(${PROJECT_NAME}_${TEST_NAME} ${TEST_SOURCE})
    ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} gtest_main ${PROJECT_NAME})
  endforeach()
endif()
