# dep/LogOutput/core_dataout/CMakeLists.txt
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
# if(rviz_QT_VERSION VERSION_LESS "5")
#   message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#   find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui QtSql QtPrintSupport)
#   ## pull in all required include dirs, define QT_LIBRARIES, etc.
#   include(${QT_USE_FILE})
# else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets Gui Sql PrintSupport)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets Qt5::Gui Qt5::Sql Qt5::PrintSupport)
# endif()
# 设置当前子目录的源文件
set(CORE_DATAOUT_SOURCES
    datacsv.cpp
    datahelper.cpp
    dataother.cpp
    dataprint.cpp
    datareport.cpp
    dataxls.cpp
)
set(CORE_DATAOUT_HEADS
    datahead.h
    datastruct.h
)
# 生成静态库或共享库，取决于你的需求
add_library(core_dataout SHARED ${CORE_DATAOUT_SOURCES} ${CORE_DATAOUT_HEADS})
set_target_properties(core_dataout PROPERTIES AUTOMOC TRUE)

# 添加当前子目录的头文件路径
target_include_directories(core_dataout PUBLIC ${CMAKE_SOURCE_DIR}/dep/LogOutput/core_dataout)

# 如果有其他依赖，可以添加链接库
target_link_libraries(core_dataout ${QT_LIBRARIES})

