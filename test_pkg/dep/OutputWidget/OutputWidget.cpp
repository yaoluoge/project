#include "OutputWidget.h"
#include <QDebug>
#include <QHeaderView>
#include <QClipboard>
#include <QApplication>
#include <QDateTime>
#include <qguiapplication.h>

#define DEFAULT_SECTION_SIZE (18)

OutputWidget::OutputWidget(QWidget *parent) : QTableWidget(parent), report_index_{0}
{
    setShowGrid(false);
    setFocusPolicy(Qt::NoFocus);
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::SingleSelection);
    setEditTriggers(QAbstractItemView::NoEditTriggers);

    this->setColumnCount(5);
    this->setHorizontalHeaderLabels(QStringList() << u8"时间" << u8"内容" << u8"位置" << u8"帧索引" << u8"缺陷值");
    this->setColumnWidth(0, 200);
    // this->setColumnWidth(1, 270);
    this->setColumnWidth(2, 150);
    // this->setColumnWidth(4, 250);

    verticalHeader()->setDefaultSectionSize(DEFAULT_SECTION_SIZE);

    QHeaderView *header = horizontalHeader();
    header->setHighlightSections(false);
    connect(header, &QHeaderView::sectionResized, [=]()
            { 
        QTableWidgetItem *currentItem = this->currentItem();
        if (!currentItem)
        {
            return;
        }
        resizeRowToContents(this->row(currentItem)); });

    connect(this, &OutputWidget::cellDoubleClicked, [&](const int &row, const int &column)
            { setCurrentCell(row, column); 
              QApplication::clipboard()->setText(item(row, column)->text()); });
    connect(this, &OutputWidget::currentItemChanged, [&](QTableWidgetItem *current, QTableWidgetItem *previous)
            { onCurrentItemChanged(current, previous); });
    connect(this, &OutputWidget::itemPressed, this, [&](QTableWidgetItem *item)
            { setCurrentItem(item); });

    copyAction_ = new QAction(u8"复制", this);
    connect(copyAction_, &QAction::triggered, [&]()
            {
        QClipboard *clipboard = QGuiApplication::clipboard();
        // clipboard->setText(selectedItems().isEmpty() ? "" : selectedItems().first()->text()); });
        clipboard->setText(selectedItems().isEmpty() ? "" : [&](){
            QString text;
                for (auto item : selectedItems())
                {
                    text += item->text() + "\t";
                }
                return text;
            }()); });
    contextMenu_.addAction(copyAction_);
    selectAllAction_ = new QAction(u8"全选", this);
    connect(selectAllAction_, &QAction::triggered, [&]()
            { selectAll(); });
    contextMenu_.addAction(selectAllAction_);
}

void OutputWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        showCustomContextMenu(event->pos());
    }
    QTableWidget::mousePressEvent(event);
}

void OutputWidget::showCustomContextMenu(const QPoint &pos)
{
    contextMenu_.exec(mapToGlobal(pos));
}

void OutputWidget::onCurrentItemChanged(QTableWidgetItem *current, QTableWidgetItem *previous)
{
    if (previous)
    {
        setRowHeight(this->row(previous), DEFAULT_SECTION_SIZE);
    }
    resizeRowToContents(this->row(current));
}

void OutputWidget::setLogError(const QString &log_content,
                               const QString &dis,
                               const QString &idx,
                               const QString &link)
{
    setLog(report_index_, log_content, false, "", dis, idx, link);
}

void OutputWidget::setLogInfo(const QString &log_content,
                              const QString &dis,
                              const QString &idx,
                              const QString &link)
{
    setLog(report_index_, log_content, true, "", dis, idx, link);
}

inline void OutputWidget::setLog(const uint &row,
                                 const QString &log_content,
                                 const bool &is_normal,
                                 const QString &content_color,
                                 const QString &dis,
                                 const QString &idx,
                                 const QString &link)
{
    if (!is_normal)
        const_cast<QString &>(content_color) = "red";
    this->setRowCount(row + 1);
    QTableWidgetItem *item_time = new QTableWidgetItem();
    QDateTime time{QDateTime::currentDateTime()};
    item_time->setText(time.toString("yyyy-MM-dd hh:mm:ss.zzz"));
    item_time->setTextColor(QColor(content_color));
    item_time->setTextAlignment(Qt::AlignCenter);
    QTableWidgetItem *item_content = new QTableWidgetItem();
    item_content->setText(log_content);
    item_content->setTextColor(QColor(content_color));
    item_content->setTextAlignment(Qt::AlignCenter);
    QTableWidgetItem *item_distance = new QTableWidgetItem();
    item_distance->setText(dis);
    item_distance->setTextColor(QColor(content_color));
    item_distance->setTextAlignment(Qt::AlignCenter);
    QTableWidgetItem *item_index = new QTableWidgetItem();
    item_index->setText(idx);
    item_index->setTextColor(QColor(content_color));
    item_index->setTextAlignment(Qt::AlignCenter);
    QTableWidgetItem *item_link = new QTableWidgetItem();
    const QString link_path{link};
    item_link->setText(link_path);
    item_link->setTextColor(QColor(content_color));
    item_link->setTextAlignment(Qt::AlignCenter);
    this->setItem(row, 0, item_time);
    this->setItem(row, 1, item_content);
    this->setItem(row, 2, item_distance);
    this->setItem(row, 3, item_index);
    this->setItem(row, 4, item_link);
    this->scrollToBottom();
    ++report_index_;
}