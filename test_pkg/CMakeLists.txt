cmake_minimum_required(VERSION 3.0.2)
project(flatness_detect)

set(CMAKE_CXX_STANDARD 14)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             rviz
             rviz_visual_tools
             geometry_msgs
             message_generation
             pcl_conversions)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} ${OPENMP_C_FLAGS})
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${OPENMP_CXX_FLAGS})
  set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)

# This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
# if(rviz_QT_VERSION VERSION_LESS "5") message(STATUS "Using Qt4 based on the
# rviz_QT_VERSION: ${rviz_QT_VERSION}") find_package(Qt4 ${rviz_QT_VERSION}
# EXACT REQUIRED QtCore QtGui) ## pull in all required include dirs, define
# QT_LIBRARIES, etc. include(${QT_USE_FILE}) else()
message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(
  Qt5
  ${rviz_QT_VERSION}
  EXACT
  REQUIRED
  Core
  Widgets
  Sql
  Network)
# make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt5::Widgets Qt5::Sql Qt5::Network Qt5::Core Qt5::Gui)
# endif()

add_subdirectory(dep/LogOutput/core_dataout)
add_subdirectory(dep/LogOutput/core_base)
add_subdirectory(dep/LogOutput/dataout)
add_subdirectory(dep/OutputWidget)
include_directories(dep)

add_message_files(FILES planeToleranceMsg.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x
# -std=c++14 -fopenmp")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  message_runtime
  geometry_msgs
  rviz_visual_tools
  pcl_conversions)

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

add_executable(${PROJECT_NAME} src/main_quadruple_cam_c2w.cpp)

add_executable(automove src/detect_automove.cpp)

add_executable(PEC src/plane_extrin_calibration_c2w.cpp)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${QT_LIBRARIES}
                      ${catkin_LIBRARIES} -fopenmp OpenMP::OpenMP_CXX)

target_link_libraries(PEC ${PCL_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

target_link_libraries(automove ${PCL_LIBRARIES} ${QT_LIBRARIES}
                      ${catkin_LIBRARIES} -fopenmp OpenMP::OpenMP_CXX)

# add_library(OutputWidget src/OutputWidget.cpp)
# set_target_properties(OutputWidget PROPERTIES AUTOMOC TRUE)
# target_link_libraries(OutputWidget ${PCL_LIBRARIES} ${catkin_LIBRARIES}
# ${QT_LIBRARIES})


add_executable(main_window src/main_window.cpp)
set_target_properties(main_window PROPERTIES AUTOMOC TRUE)
target_link_libraries(
  main_window
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  core_dataout
  core_base
  dataout
  OutputWidget)
