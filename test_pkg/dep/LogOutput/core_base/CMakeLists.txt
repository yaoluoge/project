# dep/LogOutput/core_base/CMakeLists.txt
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
# if(rviz_QT_VERSION VERSION_LESS "5")
#   message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#   find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
#   ## pull in all required include dirs, define QT_LIBRARIES, etc.
#   include(${QT_USE_FILE})
# else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets Sql Network)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets Qt5::Sql Qt5::Network)
# endif()
# 设置当前子目录的源文件
set(CORE_BASE_SOURCES
    appdata.cpp
    appinit.cpp
    quihelper.cpp
    base64helper.cpp
    iconhelper.cpp
)
set(CORE_BASE_HEADS
    datahead.h
    datastruct.h
    ${CMAKE_SOURCE_DIR}/dep/LogOutput/include/head.h
)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

# 生成静态库或共享库，取决于你的需求
add_library(core_base SHARED ${CORE_BASE_SOURCES})
set_target_properties(core_base PROPERTIES AUTOMOC TRUE)

target_include_directories(core_base PUBLIC ${CMAKE_SOURCE_DIR}/dep/LogOutput/core_base ${CORE_BASE_HEADS})
# 如果有其他依赖，可以添加链接库
target_link_libraries(core_base PUBLIC dataout ${QT_LIBRARIES})
