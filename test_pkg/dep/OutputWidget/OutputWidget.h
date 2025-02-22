#ifndef OUTPUTWIDGET_H
#define OUTPUTWIDGET_H

#include <QTableWidget>
#include <QAction>
#include <QMenu>
#include <QMouseEvent>

class OutputWidget : public QTableWidget
{
    Q_OBJECT
public:
    explicit OutputWidget(QWidget *parent = nullptr);
    void setLogError(const QString &log_content,
                     const QString &dis = u8"nan",
                     const QString &idx = u8"nan",
                     const QString &link = u8"nan");
    void setLogInfo(const QString &log_content,
                    const QString &dis = "",
                    const QString &idx = "",
                    const QString &link = "");

private:
    inline void setLog(const uint &row,
                       const QString &log_content = "",
                       const bool &is_normal = true,
                       const QString &content_color = "",
                       const QString &dis = "",
                       const QString &idx = "",
                       const QString &link = "");

protected:
    virtual void mousePressEvent(QMouseEvent *event) override;

private slots:
    void onCurrentItemChanged(QTableWidgetItem *current, QTableWidgetItem *previous);
    void showCustomContextMenu(const QPoint &pos);

private:
    QAction *copyAction_, *selectAllAction_;
    QMenu contextMenu_{this};
    uint report_index_;
};

#endif // OUTPUTWIDGET_H
