QT += sql
greaterThan(QT_MAJOR_VERSION, 4): QT += printsupport
greaterThan(QT_MAJOR_VERSION, 5): QT += core5compat

HEADERS += \
    $$PWD/datacsv.h \
    $$PWD/datahead.h \
    $$PWD/datahelper.h \
    $$PWD/dataother.h \
    $$PWD/dataprint.h \
    $$PWD/datareport.h \
    $$PWD/datastruct.h \
    $$PWD/dataxls.h

SOURCES += \
    $$PWD/datacsv.cpp \
    $$PWD/datahelper.cpp \
    $$PWD/dataother.cpp \
    $$PWD/dataprint.cpp \
    $$PWD/datareport.cpp \
    $$PWD/dataxls.cpp
