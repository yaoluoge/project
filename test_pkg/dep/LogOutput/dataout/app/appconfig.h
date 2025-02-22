#ifndef APPCONFIG_H
#define APPCONFIG_H

//#include "head.h"
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

class AppConfig
{
public:
    static QString ConfigFile;      //配置文件文件路径及名称

    //基本参数
    static int IndexDemo;           //最后选中的窗体索引
    static int LeftWidth;           //左侧宽度
    static int RightWidth;          //右侧宽度

    static int CountXls;            //导出xls记录数
    static int CountPdf;            //导出pdf记录数
    static int CountPrint;          //打印数据记录数

    static int CountThread;         //线程示例每次插入数量
    static int ColumnThread;        //线程示例列数
    static int RowThread;           //线程示例总行数
    static int TypeThread;          //线程示例类型 xls pdf

    static void readConfig();       //读取配置文件,在main函数最开始加载程序载入
    static void writeConfig();      //写入配置文件,在更改配置文件程序关闭时调用
};

#endif // APPCONFIG_H
