#ifndef DATAREPORT_H
#define DATAREPORT_H

#include "datahead.h"

class DataReport
{
public:
    // 创建无人机报表
    static void creatUavsReportHead(QStringList &list, const UavsReportData &reportData);
    static void creatUavsReportBody(QStringList &list, const UavsReportData &reportData, int imageWidth = 300);

    // 创建个人信息报表
    static void creatPersonReportHead(QStringList &list, const PersonReportData &reportData);
    static void creatPersonReportBody(QStringList &list, const PersonReportData &reportData, const QStringList &columnValue, int brCount);
    inline static void setTempImagePath(const QString &path) { path_ = path + "/"; }

private:
    static QString path_; // 临时图片路径
    // 可以自行增加其他模板数据
};

#endif // DATAREPORT_H
