cmake_minimum_required(VERSION 3.0)

PROJECT(pybullet_cpp_sharedmemory)

cmake_minimum_required(VERSION 3.12)

SUBDIRS(src)

include_directories("${PROJECT_SOURCE_DIR}/libs")
include_directories("${PROJECT_SOURCE_DIR}/libs/Eigen")
include_directories("${PROJECT_SOURCE_DIR}/libs/myGEOMETRY")
include_directories("${PROJECT_SOURCE_DIR}/src")

#add_executable(pybullet_cpp_example pybullet_cpp_example.cpp)

#target_link_libraries(pybullet_cpp_example PUBLIC pybullet_cpp "${PROJECT_SOURCE_DIR}/libs/virtuoseDLL.lib")


# Create executable for each example
# Look for examples under folders
file(GLOB EXE_LIST RELATIVE ${PROJECT_SOURCE_DIR} "[0-9]*-*/[0-9]*.cpp")
foreach ( SRC_FILE ${EXE_LIST} )

  string(REPLACE ".cpp" "" TARGET_EXE_NAME ${SRC_FILE})
  string(REPLACE "/" "_" TARGET_EXE_NAME ${TARGET_EXE_NAME})
  
  MESSAGE("creating TARGET_EXE_NAME: '${TARGET_EXE_NAME}'")
  
  add_executable(${TARGET_EXE_NAME} ${SRC_FILE})
  target_link_libraries(${TARGET_EXE_NAME}  PUBLIC pybullet_cpp "${PROJECT_SOURCE_DIR}/libs/virtuoseDLL.lib")

endforeach()