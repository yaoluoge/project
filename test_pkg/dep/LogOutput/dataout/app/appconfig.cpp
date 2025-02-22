#include "appconfig.h"

QString AppConfig::ConfigFile = "config.ini";

int AppConfig::IndexDemo = 0;
int AppConfig::LeftWidth = 150;
int AppConfig::RightWidth = 160;

int AppConfig::CountXls = 10000;
int AppConfig::CountPdf = 500;
int AppConfig::CountPrint = 3000;

int AppConfig::CountThread = 1000;
int AppConfig::ColumnThread = 10;
int AppConfig::RowThread = 10000;
int AppConfig::TypeThread = 0;

void AppConfig::readConfig()
{
    QSettings set(ConfigFile, QSettings::IniFormat);
#if (QT_VERSION < QT_VERSION_CHECK(6,0,0))
    set.setIniCodec("utf-8");
#endif

    set.beginGroup("AppConfig");
    IndexDemo = set.value("IndexDemo", IndexDemo).toInt();
    LeftWidth = set.value("LeftWidth", LeftWidth).toInt();
    RightWidth = set.value("RightWidth", RightWidth).toInt();

    CountXls = set.value("CountXls", CountXls).toInt();
    CountPdf = set.value("CountPdf", CountPdf).toInt();
    CountPrint = set.value("CountPrint", CountPrint).toInt();

    CountThread = set.value("CountThread", CountThread).toInt();
    ColumnThread = set.value("ColumnThread", ColumnThread).toInt();
    RowThread = set.value("RowThread", RowThread).toInt();
    TypeThread = set.value("TypeThread", TypeThread).toInt();
    set.endGroup();
}

void AppConfig::writeConfig()
{
    QSettings set(ConfigFile, QSettings::IniFormat);
#if (QT_VERSION < QT_VERSION_CHECK(6,0,0))
    set.setIniCodec("utf-8");
#endif

    set.beginGroup("AppConfig");
    set.setValue("IndexDemo", IndexDemo);
    set.setValue("LeftWidth", LeftWidth);
    set.setValue("RightWidth", RightWidth);

    set.setValue("CountXls", CountXls);
    set.setValue("CountPdf", CountPdf);
    set.setValue("CountPrint", CountPrint);

    set.setValue("CountThread", CountThread);
    set.setValue("ColumnThread", ColumnThread);
    set.setValue("RowThread", RowThread);
    set.setValue("TypeThread", TypeThread);
    set.endGroup();
}
