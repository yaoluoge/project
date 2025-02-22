#ifndef APPDATA_H
#define APPDATA_H

// #include "head.h"
#include <QtCore>
#include <QtGui>
#include <QtSql>

#if (QT_VERSION >= QT_VERSION_CHECK(5,0,0))
#include <QtWidgets>
#include <QtPrintSupport>
#endif

#if (QT_VERSION >= QT_VERSION_CHECK(6,0,0))
#include <QtCore5Compat>
#endif

#pragma execution_character_set("utf-8")
#define TIMEMS qPrintable(QTime::currentTime().toString("HH:mm:ss zzz"))
#define DATETIME qPrintable(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"))


class AppData
{
public:
    static QString TitleFlag;       //标题标识
    static int RowHeight;           //行高
    static int RightWidth;          //右侧宽度
    static int FormWidth;           //窗体宽度
    static int FormHeight;          //窗体高度

    static void checkRatio();       //校验分辨率    
};

#endif // APPDATA_H
