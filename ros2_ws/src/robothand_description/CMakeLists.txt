cmake_minimum_required(VERSION 3.8)
project(robothand_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
