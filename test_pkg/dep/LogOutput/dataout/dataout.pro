QT += core gui sql
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
greaterThan(QT_MAJOR_VERSION, 5): QT += core5compat

#引入charts模块(Qt>=5.7)用于演示导出和打印图表
greaterThan(QT_MAJOR_VERSION, 4) {
greaterThan(QT_MINOR_VERSION, 6) {
DEFINES += qchart
}}
greaterThan(QT_MAJOR_VERSION, 5) {
DEFINES += qchart
}

contains(DEFINES, qchart) {
QT += charts
}

TARGET      = dataout
TEMPLATE    = app

DEFINES     += no_qrc_font
RESOURCES   += main.qrc
HEADERS     += head.h
SOURCES     += main.cpp

INCLUDEPATH += $$PWD
INCLUDEPATH += $$PWD/app
INCLUDEPATH += $$PWD/form

include ($$PWD/app/app.pri)
include ($$PWD/form/form.pri)

INCLUDEPATH += $$PWD/../core_base
include ($$PWD/../core_base/core_base.pri)

#引入核心组件
INCLUDEPATH += $$PWD/../core_dataout
include ($$PWD/../core_dataout/core_dataout.pri)
