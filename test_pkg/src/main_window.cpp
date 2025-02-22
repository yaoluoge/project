#include "LogOutput/core_base/quihelper.h"
#include "LogOutput/core_dataout/datahelper.h"
#include "LogOutput/core_dataout/dataother.h"
#include "LogOutput/core_dataout/dataprint.h"
#include "LogOutput/core_dataout/datareport.h"
#include "OutputWidget/OutputWidget.h"
#include "nlohmann/json.hpp"
#include <QApplication>
#include <QCloseEvent>
#include <QColor>
#include <QComboBox>
#include <QDateTime>
#include <QDebug>
#include <QDesktopServices>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QJsonDocument>
#include <QLabel>
#include <QMessageBox>
#include <QPalette>
#include <QPointer>
#include <QProcess>
#include <QPushButton>
#include <QRegularExpression>
#include <QSharedPointer>
#include <QSizePolicy>
#include <QSlider>
#include <QStackedWidget>
#include <QTextEdit>
#include <QTimer>
#include <QToolButton>
#include <QUrl>
#include <QVBoxLayout>
#include <QWidget>
#include <eigen3/Eigen/Dense>
#include <flatness_detect/planeToleranceMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <limits>
#include <map>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rviz/default_plugin/point_cloud2_display.h>
#include <rviz/default_plugin/tools/point_tool.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/view_controller.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <thread>

using json = nlohmann::json;
namespace rviz {
class Display;
class RenderPanel;
class VisualizationManager;
} // namespace rviz

namespace flatness_detection {
class Param {
public:
  Param() : pnh_("~") { readParam(); }

  ros::NodeHandle getNodeHandle() const { return pnh_; }

  static Param &getInstance() {
    static Param instance;
    return instance;
  }

public:
  std::string icon_predfix_name;
  std::string base_path;
  std::string folder;
  std::string folder_pdf;
  std::string folder_temp;
  std::string current_task_image_folder;
  std::string cloud_points_topic;
  std::string current_task_folder_all;
  std::string height_R_topic;
  std::string plane_tolerance_topic;
  double flat_max, flat_normal;
  bool is_state_test;

private:
  void readParam() {
    pnh_.param<std::string>("icon_predfix_name", icon_predfix_name,
                            "bubble_marked_frame_");
    pnh_.param<std::string>("base_path", base_path, "/map");
    pnh_.param<std::string>("folder", folder, "icon");
    pnh_.param<std::string>("folder_pdf", folder_pdf, "pdf");
    pnh_.param<std::string>("folder_temp", folder_temp, "temp");
    pnh_.param<std::string>("cloud_points_topic", cloud_points_topic, "");
    pnh_.param<std::string>("current_task_image_folder",
                            current_task_image_folder, "");
    pnh_.param<std::string>("height_R_topic", height_R_topic, "");
    pnh_.param<std::string>("plane_tolerance_topic", plane_tolerance_topic, "");
    pnh_.param<double>("flat_max", flat_max, 0.03);
    pnh_.param<double>("flat_normal", flat_normal, 0.03);
    pnh_.param<bool>("is_state_test", is_state_test, 0.03);
  }

private:
  ros::NodeHandle pnh_;
};
} // namespace flatness_detection
// launch-prefix="gdb -ex run --args"

// using json = nlohmann::json;

class JsonToCsvConverter : public QWidget {
  Q_OBJECT
public:
  JsonToCsvConverter(QWidget *parent = nullptr) : QWidget(parent) {
    setWindowTitle("JSON to CSV Converter");

    convertButton = new QPushButton("Convert JSON to CSV");
    connect(convertButton, &QPushButton::clicked, this,
            &JsonToCsvConverter::convertJsonToCsv);

    layout = new QVBoxLayout();
    layout->addWidget(convertButton);

    setLayout(layout);
  }

private slots:
  void convertJsonToCsv() {
    QString jsonFilePath = QFileDialog::getOpenFileName(
        this, "Select JSON File", QDir::currentPath(), "JSON Files (*.json)");
    if (jsonFilePath.isEmpty())
      return;

    QFile jsonFile(jsonFilePath);
    if (!jsonFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
      qDebug() << "Failed to open JSON file.";
      return;
    }

    QString jsonData = jsonFile.readAll();
    jsonFile.close();

    json jsonObj = json::parse(jsonData.toStdString());

    QTableWidget *tableWidget = new QTableWidget();
    tableWidget->setColumnCount(2);
    tableWidget->setHorizontalHeaderLabels(
        QStringList() << "detection_time" << "flatness_tolerance");

    json flatnessArray = jsonObj["平面度检测_2024_10-10_15:58_39"];
    for (size_t i = 0; i < flatnessArray.size(); ++i) {
      tableWidget->insertRow(i);
      QTableWidgetItem *timeItem = new QTableWidgetItem(
          QString::fromStdString(flatnessArray[i]["detection_time"]));
      QTableWidgetItem *toleranceItem = new QTableWidgetItem(
          QString::fromStdString(flatnessArray[i]["flatness_tolerance"]));
      tableWidget->setItem(i, 0, timeItem);
      tableWidget->setItem(i, 1, toleranceItem);
    }

    QString csvFilePath = QFileDialog::getSaveFileName(
        this, "Save as CSV", QDir::currentPath(), "CSV Files (*.csv)");
    if (csvFilePath.isEmpty())
      return;

    QFile csvFile(csvFilePath);
    if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
      qDebug() << "Failed to open CSV file for writing.";
      return;
    }

    QTextStream out(&csvFile);
    out << "detection_time,flatness_tolerance" << endl;
    for (int i = 0; i < tableWidget->rowCount(); ++i) {
      out << tableWidget->item(i, 0)->text() << ","
          << tableWidget->item(i, 1)->text() << endl;
    }

    csvFile.close();
  }

private:
  QPushButton *convertButton;
  QVBoxLayout *layout;
};

class LogOutWidget : public QWidget {
  Q_OBJECT
  enum class Key : int { PRINT, OUT, TASK_TIME, DIS, FRAME_ID };

public:
  LogOutWidget(QWidget *parent = nullptr)
      : QWidget(parent), param_{flatness_detection::Param::getInstance()} {
    setAttribute(Qt::WA_QuitOnClose, false);
    DataHelper::init();
    readParam();
    auto pdf_path{QString::fromStdString(base_path_ + "/" + folder_pdf_)};
    auto temp_image_path{pdf_path + QString::fromStdString("/" + folder_temp_)};
    DataReport::setTempImagePath(temp_image_path);
    ROS_WARN_STREAM(base_path_ + "/" + folder_pdf_ + "/" + folder_temp_);
    connect(DataPrint::dataPrint, &DataPrint::receiveHtml,
            [&](const QString &html) { text_edit_->setHtml(html); });
    setFolderPath(base_path_, folder_pdf_);
    setFolderPath(pdf_path.toStdString(), folder_temp_);
    folder_temp_path_ = temp_image_path.toStdString();
    initform();
    connect(btn_out_.get(), &QPushButton::clicked, [&]() {
      // current_task_image_folder_ = param_.current_task_image_folder;
      current_task_image_folder_ = param_.current_task_folder_all;
      if (current_task_image_folder_.empty()) {
        QMessageBox::warning(this, u8"警告", u8"当前任务未开始，无法输出日志");
        return;
      }
      if (!QDir(QString::fromStdString(base_path_ + "/" + folder_ + "/" +
                                       current_task_image_folder_))
               .exists()) {
        QMessageBox::warning(this, u8"警告", u8"当前任务文件夹不存在");
        return;
      }
      dataOutType(Key::OUT);
    });
    connect(btn_print_.get(), &QPushButton::clicked, [&]() {
      auto image_path = QFileDialog::getExistingDirectory(
          this, u8"选择日志文件",
          QString::fromStdString(base_path_ + "/" + folder_));
      ROS_WARN_STREAM(image_path.toStdString());
      if (image_path.isEmpty()) {
        QMessageBox::warning(this, u8"警告", u8"未选择日志文件");
        return;
      }
      auto image_folder_name{QDir(image_path).dirName()};
      ROS_WARN_STREAM(image_folder_name.toStdString());
      dataOutType(Key::PRINT, image_folder_name.toStdString());
    });
    connect(btn_clear_.get(), &QPushButton::clicked,
            [&]() { text_edit_->clear(); });
  }

  virtual ~LogOutWidget() {}

  inline std::string getTempFolderPath() { return folder_temp_path_; }

private:
  inline void readParam() {
    icon_predfix_name_ = param_.icon_predfix_name;
    base_path_ = param_.base_path;
    folder_ = param_.folder;
    folder_pdf_ = param_.folder_pdf;
    folder_temp_ = param_.folder_temp;
    folder_ = param_.folder;
  }

  inline void initform() {
    QVBoxLayout *log_layout = new QVBoxLayout();
    text_edit_ = QSharedPointer<QTextEdit>(new QTextEdit());
    log_layout->addWidget(text_edit_.get());
    text_edit_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    text_edit_->setMinimumSize(730, 500);
    frame_btn_ = QSharedPointer<QFrame>(new QFrame());
    QHBoxLayout *btn_layout = new QHBoxLayout();
    frame_btn_->setLayout(btn_layout);
    btn_out_ = QSharedPointer<QPushButton>(new QPushButton(u8"输出当前日志"));
    btn_layout->addWidget(btn_out_.get());
    btn_out_tasks_ =
        QSharedPointer<QPushButton>(new QPushButton(u8"输出需要日志"));
    btn_layout->addWidget(btn_out_tasks_.get());
    connect(btn_out_tasks_.get(), &QPushButton::clicked, [&]() {
      auto image_path = QFileDialog::getExistingDirectory(
          this, u8"选择日志文件",
          QString::fromStdString(base_path_ + "/" + folder_));
      ROS_WARN_STREAM(image_path.toStdString());
      if (image_path.isEmpty()) {
        QMessageBox::warning(this, u8"警告", u8"未选择日志文件");
        return;
      }
      auto image_folder_name{QDir(image_path).dirName()};
      ROS_WARN_STREAM(image_folder_name.toStdString());
      dataOutType(Key::OUT, image_folder_name.toStdString());
    });
    btn_print_ = QSharedPointer<QPushButton>(new QPushButton(u8"打印PDF日志"));
    btn_layout->addWidget(btn_print_.get());
    btn_clear_ = QSharedPointer<QPushButton>(new QPushButton(u8"清空日志"));
    btn_layout->addWidget(btn_clear_.get());
    log_layout->addWidget(frame_btn_.get());
    setLayout(log_layout);
  }

  inline void dataOutType(const Key &type,
                          const std::string &current_task_image_folder = "") {
    DataContent dataContent;
    dataContent.fileName =
        QString::fromStdString(base_path_ + "/" + folder_pdf_ + "/dataout.pdf");
    dataContent.pageMargin = QMargins(15, 20, 15, 20);
    DataPrint::dataPrint->init();
    DataPrint::dataPrint->setDataContent(dataContent);

    // 这里重新设置html用到了打印区域所有需要设置一遍 setDataContent
    // 后再重新设置 第一次 setDataContent 后才能获取到准确的打印区域
    UavsReportData reportData;
    dataContent.html =
        getAirDetectionReportHtml(reportData, current_task_image_folder);
    if (dataContent.html == "nan")
      return;
    DataPrint::dataPrint->setDataContent(dataContent);

    if (type == Key::PRINT) {
      DataPrint::dataPrint->print();
    } else if (type == Key::OUT) {
      DataPrint::dataPrint->open();
      DataPrint::dataPrint->close();
      QUIHelper::openFile(dataContent.fileName, "导出告警报告");
    }
  }

  QString
  getAirDetectionReportHtml(UavsReportData &reportData,
                            const std::string &current_task_image_folder = "") {
    QString db_path;
    if (current_task_image_folder == "")
      const_cast<std::string &>(current_task_image_folder) =
          current_task_image_folder_;
    db_path = QString::fromStdString(base_path_ + "/" + folder_ + "/" +
                                     current_task_image_folder);
    QDir dir(db_path);
    auto file_list = dir.entryList(QDir::Files);
    QRegularExpression regex("(\\d+)");
    std::vector<std::string> image_name_list;
    for (const auto &file : file_list) {
      // 使用正则表达式匹配文件名
      QRegularExpressionMatch match = regex.match(file);
      if (match.hasMatch()) {
        // 提取捕获组中的数字
        QString number_string = match.captured(1);
        bool conversion_ok;
        int number = number_string.toInt(&conversion_ok);
        // 检查转换是否成功
        if (conversion_ok) {
          image_name_list.emplace_back(icon_predfix_name_ +
                                       std::to_string(number) + ".jpg");
          // ROS_WARN_STREAM(icon_predfix_name_ + std::to_string(number) +
          // ".jpg");
        }
      }
    }
    QStringList images;
    for (const auto &image_name : image_name_list) {
      images << db_path + "/" + QString::fromStdString(image_name);
      auto image = db_path + "/" + QString::fromStdString(image_name);
      // ROS_WARN_STREAM(image.toStdString());
    }
    // 还有很多参数可以设置,每次要变动的就是这个报表的数据
    reportData.title = QString::fromStdString("气泡检测报告");
    reportData.name = QString::fromStdString("气泡检测机器人");
    reportData.type = QString::fromStdString("焊缝、焊缝气泡");
    reportData.textCount = QString::fromStdString("1");
    reportData.textLevel = QString::fromStdString("\\");
    reportData.images = images;
    // reportData.timeAlarm =
    // getAirDetectionTime(QString::fromStdString(current_task_image_folder));
    reportData.dis_list = readJsonInfo(base_path_ + "/" + folder_,
                                       current_task_image_folder, Key::DIS);
    reportData.frame_id_list = readJsonInfo(
        base_path_ + "/" + folder_, current_task_image_folder, Key::FRAME_ID);
    reportData.timeAlarm =
        readJsonInfo(base_path_ + "/" + folder_, current_task_image_folder,
                     Key::TASK_TIME)[0];
    ROS_ERROR_STREAM(base_path_ + "/" + folder_ + "/" +
                     current_task_image_folder);
    if (reportData.timeAlarm == "nan" | reportData.dis_list[0] == "nan" |
        reportData.frame_id_list[0] == "nan")
      return "nan";
    // ROS_WARN_STREAM(images.join("").toStdString());
    // 获取打印区域的尺寸用于计算图片的宽度
    // 多减一点是为了有点冗余
    QRectF rect =
        DataPrint::dataPrint->getPrinter()->pageRect(QPrinter::DevicePixel);
    int imageWidth = (rect.width() - 40) / 2;

    QStringList list;
    DataReport::creatUavsReportHead(list, reportData);
    DataReport::creatUavsReportBody(list, reportData, imageWidth);
    return list.join("");
  }

  // 利用文件夹名称获取任务时间，仅限文件夹名称为"yyyy_MM-dd_HH:mm_ss"和“xxxxx_yyyy_MM-dd_HH:mm_ss”格式
  inline QString getAirDetectionTime(const QString &input_string) {
    QRegularExpression regex;
    if (!image_folder_prefix_.empty()) {
      regex = QRegularExpression(QString::fromStdString(
          image_folder_prefix_ + "_(\\d{4})_(\\d{2}-\\d{2})_(\\d{2}):("
                                 "\\d{2})_(\\d{2})"));
    }
    regex = QRegularExpression(QString::fromStdString(
        "(\\d{4})_(\\d{2}-\\d{2})_(\\d{2}):(\\d{2})_(\\d{2})"));
    auto match = regex.match(input_string);
    if (!match.hasMatch()) {
      QMessageBox::warning(this, u8"警告",
                           u8"任务时间匹配出错,文件夹名称不正确");
      return "nan";
    }
    auto year = match.captured(1), date = match.captured(2),
         hour = match.captured(3), minute = match.captured(4),
         second = match.captured(5);
    // "yyyy-MM-dd HH:mm:ss"
    auto alarm_time{year + "-" + date + " " + hour + ":" + minute + ":" +
                    second};
    return alarm_time;
  }

  inline QStringList readJsonInfo(const std::string &json_folder_path,
                                  const std::string &json_name,
                                  const Key &key) {
    QDir dir{QString::fromStdString(json_folder_path + "/" + json_name)};
    dir.setNameFilters(QStringList() << "*.json");
    auto json_list = dir.entryList(QDir::Files | QDir::Hidden);
    if (json_list.isEmpty()) {
      QMessageBox::warning(this, u8"警告", u8"json文件不存在");
      return QStringList() << "nan";
    } else if (json_list.size() > 1) {
      QMessageBox::warning(this, u8"警告", u8"json文件过多");
      return QStringList() << "nan";
    }
    std::ifstream file{json_folder_path + "/" + json_name + "/" +
                       json_list[0].toStdString()};
    if (!file.is_open()) {
      QMessageBox::warning(this, u8"警告", u8"json文件打开失败");
      return QStringList() << "nan";
    }
    json j_get;
    try {
      file >> j_get;
    } catch (const json::parse_error &e) {
      QMessageBox::warning(this, u8"警告", u8"json文件解析失败");
      Q_EMIT sender_errorLog("json解析错误:" +
                             QString::fromStdString(e.what()));
      return QStringList() << "nan";
    }
    QStringList ret;
    switch (key) {
    case Key::TASK_TIME:
      ret << getAirDetectionAlarmTime(j_get["task_time"].get<std::string>());
      break;
    case Key::DIS:
      for (const auto &j : j_get[json_name]) {
        ret << QString::number(j["distance"], 'f', 4);
      }
      break;
    case Key::FRAME_ID:
      for (const auto &j : j_get[json_name]) {
        ret << QString::number(static_cast<int>(j["index"]));
      }
      break;

    default:
      ret << "nan";
      break;
    }
    return ret;
  }

  inline QString getAirDetectionAlarmTime(const std::string &input_string) {
    auto regex{QRegularExpression(QString::fromStdString(
        "(\\d{4})_(\\d{2}-\\d{2})_(\\d{2}):(\\d{2})_(\\d{2})"))};
    auto match = regex.match(QString::fromStdString(input_string));
    if (!match.hasMatch()) {
      QMessageBox::warning(this, u8"警告", u8"任务时间解析出错,时间格式不正确");
      return "nan";
    }
    auto year = match.captured(1), date = match.captured(2),
         hour = match.captured(3), minute = match.captured(4),
         second = match.captured(5);
    // "yyyy-MM-dd HH:mm:ss"
    auto alarm_time{year + "-" + date + " " + hour + ":" + minute + ":" +
                    second};
    return alarm_time;
  }

  inline void setFolderPath(const std::string &base_path,
                            const std::string &folder_path) {
    auto icon_folder_path{base_path + "/" + folder_path};
    QDir dir(QString::fromStdString(icon_folder_path));
    if (!dir.exists()) {
      if (!dir.mkpath(QString::fromStdString(icon_folder_path))) {
        // setLogError(u8"创建icon文件夹失败:" +
        // QString::fromStdString(icon_folder_path));
        Q_EMIT sender_errorLog(u8"创建icon文件夹失败:" +
                               QString::fromStdString(icon_folder_path));
        qWarning() << "Failed to create icon folder.";
        return;
      }
    }
  }

Q_SIGNALS:
  void sender_errorLog(const QString &log);

private:
  QSharedPointer<QTextEdit> text_edit_;
  QSharedPointer<QFrame> frame_btn_;
  QSharedPointer<QPushButton> btn_out_, btn_out_tasks_, btn_print_, btn_clear_;
  std::string icon_predfix_name_;
  std::string folder_;
  std::string base_path_;
  std::string folder_pdf_;
  std::string folder_temp_, folder_temp_path_;
  std::string current_task_image_folder_;
  std::string image_folder_prefix_;
  flatness_detection::Param &param_;
};

// typedef struct WidgetParam
// {
//     double
// };

class FlatDetectionWidegt : public QWidget {
  Q_OBJECT
public:
  enum WebBtnMsgsEnum : int { CAM_CALIBAR, MOTION_CTRL, LOG_OUTPUT };

  FlatDetectionWidegt(QWidget *parent = nullptr)
      : QWidget(parent), nh_(""),
        param_{flatness_detection::Param::getInstance()},
        log_out_widget_{QSharedPointer<LogOutWidget>(new LogOutWidget())},
        output_widget_{QSharedPointer<OutputWidget>(new OutputWidget())},
        detect_count_{0}, report_index_{0}, is_set_task_folder_{false} {
    readParam();
    pub_cam_calibar_msgs_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/cam_calibar_msgs", 10);
    sub_height_delta_R_ = nh_.subscribe<flatness_detect::planeToleranceMsg>(
        height_R_topic_.toStdString(), 1,
        [&](const flatness_detect::planeToleranceMsg::ConstPtr &msg) {
          this->getHeightDeltaR_cb(msg, pub_cam_calibar_msgs_);
        });
    sub_plane_tolerance_ = nh_.subscribe<flatness_detect::planeToleranceMsg>(
        plane_tolerance_topic_.toStdString(), 1,
        [this](const flatness_detect::planeToleranceMsg::ConstPtr &msg) {
          auto state_auto{msg->mode};
          // 后续删除
          if (param_.is_state_test)
            state_auto = true;
          // 后续删除
          if (state_auto == false) // 手动模式
          {
            auto stamp{QString::number(msg->header.stamp.toNSec() / 100000000) +
                       "." +
                       QString::number(msg->header.stamp.toNSec() % 100000000)};
            detect_time_map_[msg->plane_tolerance] = stamp;
            max_plane_tolerance_ = *std::max_element(detect_time_map_.begin(),
                                                     detect_time_map_.end());
            /***桌面端显示*/
            // lbl_plane_tolerance_->setNum(msg->plane_tolerance);
            // lbl_max_plane_tolerance_->setNum(max_plane_tolerance_.first);
            // auto time{QDateTime::currentDateTime().toString(
            //     "yyyy-MM-dd hh:mm:ss.zzz")};
            // lbl_stamp_->setText(u8"时间戳: " + time);
            // 可能出问题
            ++detect_count_;
            // lbl_detect_count_->setNum(static_cast<int>(detect_count_));
            // 可能出问题
            /***桌面端显示*/
            return;
          } else {
            if (msg->plane_tolerance >= param_.flat_normal) {
              auto thread{std::thread([&]() {
                this->getPlaneTolerance_cb(
                    msg, std::ref(this->max_plane_tolerance_mutex_));
              })};
              if (thread.joinable())
                thread.join();
            } else {
              ROS_INFO_STREAM("[flatness_detect] plane_tolerance: ");
              auto stamp{
                  QString::number(msg->header.stamp.toNSec() / 100000000) +
                  "." +
                  QString::number(msg->header.stamp.toNSec() % 100000000)};
              detect_time_map_[msg->plane_tolerance] = stamp;
              max_plane_tolerance_ = *std::max_element(detect_time_map_.begin(),
                                                       detect_time_map_.end());
            }
            /***桌面端显示*/
            // lbl_plane_tolerance_->setNum(msg->plane_tolerance);
            // lbl_max_plane_tolerance_->setNum(max_plane_tolerance_.first);
            /***桌面端显示*/
            // lbl_stamp_->setText(u8"时间戳: " + max_plane_tolerance_.second);
            auto time{QDateTime::currentDateTime().toString(
                "yyyy-MM-dd hh:mm:ss.zzz")};
            /***桌面端显示*/
            // lbl_stamp_->setText(u8"时间戳: " + time);
            /***桌面端显示*/
            /**暂时注释*/
            if (msg->defect_location_y.empty())
              return;
            auto location{
                "x= " +
                QString::number(msg->defect_location_x[get_num_], 'f', 2) +
                " y= " + QString::number(msg->defect_location, 'f', 2)};
            ++get_num_;
            ROS_WARN_STREAM("msg->defect_location_y size: "
                            << msg->defect_location_y.size());
            if (msg->defect_location_y.size() == 0)
              this->setLogError(QString::number(msg->plane_tolerance, 'f', 2),
                                location, QString::number(msg->measure_num));
            // else if (msg->plane_tolerance >= flat_max_)
            else if (msg->plane_tolerance >= param_.flat_normal)
              this->setLogError(
                  QString::number(msg->plane_tolerance, 'f', 2), location,
                  QString::number(msg->measure_num),
                  QString::number(msg->defect_location_y[get_num_], 'f', 4));
            /**暂时注释*/
            //// setLogInfo(QString::fromStdString(msg->info_error));
            // {
            //   std::lock_guard<std::mutex> lock(
            //       this->max_plane_tolerance_mutex_);
            //   /***桌面端显示*/
            //   // lbl_detect_count_->setNum(static_cast<int>(detect_count_));
            //   /***桌面端显示*/
            // }
          }
        });
    sub_cloud_points_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        cloud_points_topic_.toStdString(), 1,
        [&](const sensor_msgs::PointCloud2::ConstPtr &msg) {
          this->disPlayCloudPoints_cb(msg);
        });
    sub_web_btn_msgs_ = nh_.subscribe<std_msgs::Float32>(
        "/web_btn_msgs", 1, [&](const std_msgs::Float32ConstPtr &msg) {
          if (WebBtnMsgsEnum::CAM_CALIBAR == static_cast<int>(msg->data)) {
            system("gnome-terminal -e 'bash -c "
                   "\"/home/abc/lyx/flat/calibration.sh; exec bash\"'");
            ROS_INFO_STREAM("\033[32;47mmsg: " << msg->data);
          } else if (WebBtnMsgsEnum::MOTION_CTRL == static_cast<int>(msg->data))
            system("gnome-terminal -e 'bash -c "
                   "\"/home/abc/lyx/robot14s/control.sh; exec bash\"'");
          else if (WebBtnMsgsEnum::LOG_OUTPUT == static_cast<int>(msg->data)) {
            emit btn_log_->clicked();
            std::this_thread::sleep_for(std::chrono::seconds(1));
            system("gnome-terminal -e 'bash -c "
                   "\"echo '123' | sudo -S /home/abc/lyx/flat/csv_commit.sh; exec bash\"'");
            ROS_INFO_STREAM("\033[32;47mmsg: " << msg->data);
          }
        });
    connect(this, &FlatDetectionWidegt::sender_setCurrentTaskFolder, [&]() {
      setCurrentTaskFolder(current_task_image_folder_);
      param_.current_task_image_folder =
          current_task_image_folder_.toStdString();
    });
    initForm();
    // json_path_ = "./.flatness_tolerance_" +
    // QDateTime::currentDateTime().toString("yyyy-MM-dd
    // hh:mm:ss.zzz").toStdString() + ".json";
    gui_update_timer = QSharedPointer<QTimer>(new QTimer(this));
    int value = 0;
    connect(gui_update_timer.get(), &QTimer::timeout, [&]() {
      this->update();
      //                     lbl_plane_tolerance_->setValue(value++);
      // if (value > 1000) {  // 设定一个结束条件
      //     gui_update_timer->stop();
      // }
    });
    gui_update_timer->start(30);
  }
  ~FlatDetectionWidegt() {}

private:
  inline void
  getHeightDeltaR_cb(const flatness_detect::planeToleranceMsgConstPtr &msg,
                     ros::Publisher &pub) {
    // spin_height_delta_->setValue(msg->height_delta);
    // spin_rot_x_->setValue(msg->R.x);
    // spin_rot_y_->setValue(msg->R.y);
    // spin_rot_z_->setValue(msg->R.z);
    // spin_rot_w_->setValue(msg->R.w);
    Eigen::Quaterniond q(msg->R.w, msg->R.x, msg->R.y, msg->R.z);
    auto r{q.normalized().toRotationMatrix()};
    text_edit_->append(u8"话题: " +
                       QString::fromStdString(msg->cam_topic_name));
    text_edit_->append(u8"旋转矩阵:\n" + QString::number(r(0, 0), 'f', 4) +
                       " " + QString::number(r(0, 1), 'f', 4) + " " +
                       QString::number(r(0, 2), 'f', 4) + "\n" +
                       QString::number(r(1, 0), 'f', 4) + " " +
                       QString::number(r(1, 1), 'f', 4) + " " +
                       QString::number(r(1, 2), 'f', 4) + "\n" +
                       QString::number(r(2, 0), 'f', 4) + " " +
                       QString::number(r(2, 1), 'f', 4) + " " +
                       QString::number(r(2, 2), 'f', 4));

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    pose_stamped.pose.position.x = 0;
    pose_stamped.pose.position.y = 0;
    pose_stamped.pose.position.z = 0;
    pose_stamped.header.frame_id = msg->cam_topic_name;
    pose_stamped.header.stamp = ros::Time::now();
    pub.publish(pose_stamped);
  }

  inline void
  disPlayCloudPoints_cb(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // ROS_WARN_STREAM("****主线程");
  }

  inline void setFolderPath(const std::string &base_path,
                            const std::string &folder_path) {
    auto _folder_path{base_path + "/" + folder_path};
    QDir dir(QString::fromStdString(_folder_path));
    if (!dir.exists()) {
      if (!dir.mkpath(QString::fromStdString(_folder_path))) {
        output_widget_->setLogError(
            QString::fromStdString(u8"创建" + folder_path + u8"文件夹失败:") +
            QString::fromStdString(_folder_path));
        qWarning() << "Failed to create icon folder.";
        // ROS_ERROR_STREAM(_folder_path);
        return;
      }
    }
  }

  inline void setLogError(const QString &log_content,
                          const QString &dis = u8"nan",
                          const QString &idx = u8"nan",
                          const QString &link = u8"nan") {
    setLog(report_index_, log_content, false, "", dis, idx, link);
  }

  inline void setLogInfo(const QString &log_content, const QString &dis = "",
                         const QString &idx = "", const QString &link = "") {
    setLog(report_index_, log_content, true, "", dis, idx, link);
  }

  inline void setLog(const uint &row, const QString &log_content = "",
                     const bool &is_normal = true,
                     const QString &content_color = "", const QString &dis = "",
                     const QString &idx = "", const QString &link = "") {
    if (!is_normal)
      const_cast<QString &>(content_color) = "red";
    output_widget_->setRowCount(row + 1);
    QTableWidgetItem *item_time = new QTableWidgetItem();
    QDateTime time = QDateTime::currentDateTime();
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
    output_widget_->setItem(row, 0, item_time);
    output_widget_->setItem(row, 1, item_content);
    output_widget_->setItem(row, 2, item_distance);
    output_widget_->setItem(row, 3, item_index);
    output_widget_->setItem(row, 4, item_link);
    output_widget_->scrollToBottom();
    ++report_index_;
  }

  inline void
  getPlaneTolerance_cb(const flatness_detect::planeToleranceMsg::ConstPtr &msg,
                       std::mutex &mutex) {
    if (!is_set_task_folder_) {
      Q_EMIT sender_setCurrentTaskFolder();
      is_set_task_folder_ = true;
      start_report_time_ = QDateTime::currentDateTime()
                               .toString("yyyy_MM-dd_hh:mm_ss")
                               .toStdString();
      j_["task_time"] = start_report_time_;
      ROS_INFO_STREAM("start_report_time_: ");
    }
    {
      std::lock_guard<std::mutex> lock(mutex); // 使用lock_guard保证线程安全

      // lbl_plane_tolerance_->setValue(msg->plane_tolerance);
      auto stamp{QString::number(msg->header.stamp.toNSec() / 100000000) + "." +
                 QString::number(msg->header.stamp.toNSec() % 100000000)};
      detect_time_map_[msg->plane_tolerance] = stamp;
      max_plane_tolerance_ =
          *std::max_element(detect_time_map_.begin(), detect_time_map_.end());
      auto time{QDateTime::currentDateTime()
                    .toString("yyyy-MM-dd hh:mm:ss.zzz")
                    .toStdString()};
      auto pair{std::make_pair(time, msg->info_error)};
      // item_.emplace_back(pair);
      auto base_path{path_ + "/" + folder_ + "/"};
      if (msg->defect_location_y.size() == 0) {
        auto defect_location_x{
            [&]() -> std::vector<std::string> { return {"nan"}; }()};
        auto defect_location_y{
            [&]() -> std::vector<std::string> { return {"nan"}; }()};
        if (!setJsonInfo(
                base_path + current_task_image_folder_.toStdString(),
                pair.first, pair.second, defect_location_x,
                QString::number(msg->defect_location, 'f', 4).toStdString(),
                defect_location_y,
                QString::number(msg->measure_num).toStdString()))
          return;
        // nan可能是未受到缺陷值问题导致的
      } else {
        auto defect_location_x{convertVector(msg->defect_location_x)};
        auto defect_location_y{convertVector(msg->defect_location_y)};
        if (!setJsonInfo(
                base_path + current_task_image_folder_.toStdString(),
                pair.first, pair.second, defect_location_x,
                QString::number(msg->defect_location, 'f', 4).toStdString(),
                defect_location_y,
                QString::number(msg->measure_num).toStdString()))
          return;
      }
      // lbl_max_plane_tolerance_->setValue(max_plane_tolerance_.first);
      // lbl_stamp_->setText(u8"时间戳: " + max_plane_tolerance_.second);
      ++detect_count_;
      // lbl_detect_count_->setValue(static_cast<int>(detect_count_));
    }
  }

  inline std::vector<std::string>
  convertVector(const std::vector<double> &double_vector) {
    std::vector<std::string> str_vector;
    for (double value : double_vector) {
      std::ostringstream oss;
      oss << value;
      str_vector.push_back(oss.str());
    }
    return str_vector;
  }

  inline bool setJsonInfo(const std::string &file_path,
                          const std::string &detection_time,
                          const std::string &flatness_tolerance,
                          const std::vector<std::string> &detection_location_x,
                          const std::string &detection_location,
                          const std::vector<std::string> &detection_location_y,
                          const std::string &measure_num) {
    json detection_location_x_json{{"x", detection_location_x},
                                   {"detect_count", detect_count_}};
    json detection_location_y_json{{"y", detection_location_y},
                                   {"detect_count", detect_count_}};
    json json_info{{"detection_time", detection_time},
                   {"flatness_tolerance", flatness_tolerance},
                   {"detection_location_x", detection_location_x_json},
                   {"detection_location", detection_location},
                   {"detection_location_y", detection_location_y_json},
                   {"measure_num", measure_num}};
    json_.emplace_back(json_info);
    j_[current_task_image_folder_.toStdString()] = json_;
    auto json_path{QString::fromStdString(file_path + "/") + "." +
                   current_task_image_folder_ + ".json"};
    QFile file(json_path);
    ROS_WARN_STREAM(json_path.toStdString());
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      // 将 JSON 文档写入文件
      file.write(j_.dump(4).c_str());
      file.close();
      chmod(json_path.toStdString().c_str(),
            S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
      return true;
      qDebug() << "JSON data written to file successfully.";
    } else {
      qDebug() << "Error opening file for writing.";
      output_widget_->setLogError(u8"JSON文件写入失败");
      return false;
    }
  }

  inline void readParam() {
    cloud_points_topic_ = QString::fromStdString(param_.cloud_points_topic);
    plane_tolerance_topic_ =
        QString::fromStdString(param_.plane_tolerance_topic);
    height_R_topic_ = QString::fromStdString(param_.height_R_topic);
    current_task_image_folder_ =
        QString::fromStdString(param_.current_task_image_folder);
    path_ = param_.base_path;
    flat_max_ = param_.flat_max;
  }

  inline void terminalInfoOutput() {
    auto output_path{ros::package::getPath("flatness_detect") +
                     "/src/.terminal_info.txt"};
    file_terminal_.setFileName(QString::fromStdString(output_path));
    if (!file_terminal_.exists())
      file_terminal_.open(QIODevice::WriteOnly | QIODevice::Text);
    else
      file_terminal_.open(QIODevice::Truncate | QIODevice::WriteOnly |
                          QIODevice::Text);
    process_.setStandardOutputFile(QString::fromStdString(output_path));
    timer_update_terminal_info_ = QSharedPointer<QTimer>(new QTimer());
    connect(timer_update_terminal_info_.get(), &QTimer::timeout, [&]() {
      if (file_terminal_.isOpen() && file_terminal_.isReadable()) {
        file_terminal_.seek(0);
        auto new_output{file_terminal_.readAll()};
        text_edit_->setPlainText(new_output);
      }
    });
    timer_update_terminal_info_->start(100);
  }

  inline void setCurrentTaskFolder(const QString &folder_name = "") {
    if (folder_name == "") {
      auto current_time{QDateTime::currentDateTime()};
      const_cast<QString &>(folder_name) =
          current_time.toString("yyyy_MM-dd_hh:mm_ss");
    }
    auto base_path{path_ + "/" + folder_};
    if (QDir(QString::fromStdString(base_path) + "/" + folder_name).exists()) {
      // param_.image_folder_prefix = folder_name.toStdString();
      // auto
      // last_name{QDateTime::currentDateTime().toString("_yyyy_MM-dd_hh:mm_ss")
      // + QString::number(report_index_)};
      auto last_name{
          QDateTime::currentDateTime().toString("_yyyy_MM-dd_hh:mm_ss")};
      const_cast<QString &>(folder_name) += last_name;
      current_task_image_folder_ = folder_name;
    }
    setFolderPath(base_path, folder_name.toStdString());
    param_.current_task_folder_all = current_task_image_folder_.toStdString();
  }

  void json_to_csv_file(const json &j, const std::string &filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "Failed to open file: " << filename << std::endl;
      return;
    }

    file << "检测时间,平面度公差,测量次数,索引\n"; // CSV header

    // Assuming the JSON is an object with a key that contains an array of
    // detections
    const auto &detections = j.at(current_task_image_folder_.toStdString());
    for (const auto &detection : detections) {
      file << detection.at("detection_time") << ","
           << detection.at("flatness_tolerance") << ","
           << detection.at("measure_num") << ","
           << detection.at("detection_location_x")["detect_count"] << "\n";
    }
    file << "检测位置X,检测位置Y,缺陷值,索引,测量次数\n"; // CSV header
    // file <<
    // "detection_location_x,detection_location,detection_location_y,detect_count,measure_num\n";
    // // CSV header

    // std::string detect_count{
    // detections[current_task_image_folder_.toStdString()]["detect_count"] };
    for (const auto &detection : detections) {
      //   file << detection.at("detection_time") << "," <<
      //   detection.at("flatness_tolerance") << ","
      //        << detection.at("measure_num") << "\n";
      int index = 0;
      for (const auto &x : detection.at("detection_location_x").at("x")) {
        file << x << "," << detection.at("detection_location") << ","
             << detection.at("detection_location_y").at("y")[index] << ","
             << detection.at("detection_location_x")["detect_count"] << ","
             << detection.at("measure_num") << "\n";
        ++index;
      }
    }
    file.close();
  }

  inline void initForm() {
    setWindowTitle(u8"平面度检测");
    terminalInfoOutput();
    rviz_render_panel_ = boost::make_shared<rviz::RenderPanel>();
    QPointer<QVBoxLayout> main_layout = new QVBoxLayout();
    QPointer<QHBoxLayout> optional_layout = new QHBoxLayout();
    output_widget_->setMinimumHeight(150);
    output_widget_->setMaximumHeight(300);
    btn_log_ = QSharedPointer<QPushButton>(new QPushButton(u8"日志输出"));
    connect(btn_log_.get(), &QPushButton::clicked, [&]() {
      //   log_out_widget_->setWindowTitle(u8"日志输出");
      //   log_out_widget_->show();
      auto json_filename{path_ + "/" + folder_ + "/" +
                         current_task_image_folder_.toStdString() + "/" + "." +
                         current_task_image_folder_.toStdString() + ".json"};
      std::ifstream json_file(json_filename);
      if (!json_file.is_open()) {
        std::cerr << "Failed to open JSON file: " << json_filename << std::endl;
        return 1;
      }
      std::string json_data((std::istreambuf_iterator<char>(json_file)),
                            std::istreambuf_iterator<char>());
      json_file.close();
      // Parse JSON data
      json j = json::parse(json_data);
      if (j.empty()) {
        QMessageBox::warning(this, u8"警告",
                             u8"无数据，确保算法已经运行或者检测到了缺陷点");
        return 1;
      }
      // json_to_csv_file(j, path_ + "/" + folder_ + "/" +
      //                         current_task_image_folder_.toStdString() + "/"
      //                         +
      //                         "/检测结果.csv"); });
      json_to_csv_file(j, path_ + "/" + "/检测结果.csv");
    });
    btn_open_robot_ =
        QSharedPointer<QPushButton>(new QPushButton(u8"机器控制"));
    connect(btn_open_robot_.get(), &QPushButton::clicked, [&]() {
      // 设置启动终端的命令
      const QString terminal_command{"gnome-terminal"};
      QDir package_path{
          QString::fromStdString(ros::package::getPath("flatness_detect"))};
      const auto arm64_exe_path{
          package_path.canonicalPath() +
          "/../../../CANopen/acusb/arm64/build/handle_testLikeCan"};
      auto arguments{QStringList() << "-e" << arm64_exe_path};
      process_.start(terminal_command, arguments);
      try {
        if (!process_.waitForStarted()) {
          QMessageBox::warning(this, u8"警告", u8"机器控制程序启动失败");
          throw std::runtime_error("机器控制程序启动失败");
        }
        if (!process_.waitForFinished()) {
          QMessageBox::warning(this, u8"警告", u8"机器控制程序执行失败");
          throw std::runtime_error("机器控制程序执行失败");
        }
      } catch (const std::runtime_error &e) {
        ROS_ERROR_STREAM(e.what());
        output_widget_->setLogError(QString::fromStdString(e.what()));
        return;
      }
    });
    connect(&process_, &QProcess::readyReadStandardOutput, [&]() {
      auto info_output{process_.readAllStandardOutput()};
      text_edit_->append(QString::fromUtf8(info_output));
    });
    spin_point_size_ = QSharedPointer<QSpinBox>(new QSpinBox());
    const auto size{3};
    spin_point_size_->setValue(size);
    QPointer<QLabel> lbl_point_size = new QLabel(u8"点云尺寸大小:");
    lbl_point_size->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    QPointer<QLabel> lbl_topic = new QLabel(u8"Fixed Frame:");
    lbl_topic->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    line_edit_ = QSharedPointer<QLineEdit>(new QLineEdit());
    const QString fixed_frame{"world"};
    line_edit_->setText(fixed_frame);
    line_edit_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    optional_layout->addWidget(btn_log_.get());
    optional_layout->addWidget(btn_open_robot_.get());
    optional_layout->addWidget(lbl_point_size);
    optional_layout->addWidget(spin_point_size_.get());
    optional_layout->addWidget(lbl_topic);
    optional_layout->addWidget(line_edit_.get());
    main_layout->addWidget(rviz_render_panel_.get());
    main_layout->addWidget(output_widget_.get());
    main_layout->addLayout(optional_layout);

    QPointer<QGridLayout> real_layout = new QGridLayout();
    rviz_visualization_manager_.reset(
        new rviz::VisualizationManager(rviz_render_panel_.get()));
    rviz_render_panel_->initialize(
        rviz_visualization_manager_->getSceneManager(),
        rviz_visualization_manager_.get());
    rviz_render_panel_->setMinimumSize(730, 500);
    rviz_visualization_manager_->initialize();
    rviz_visualization_manager_->startUpdate();
    rviz_visualization_manager_->setFixedFrame(fixed_frame);
    connect(line_edit_.get(), &QLineEdit::textChanged,
            [&](const QString &text) {
              rviz_visualization_manager_->setFixedFrame(text);
            });
    setLayout(main_layout);
    //

    grid_display_.reset(
        rviz_visualization_manager_->createDisplay("rviz/Grid", "Grid", true));
    cloud_points_display_.reset(rviz_visualization_manager_->createDisplay(
        "rviz/PointCloud2", "PointCloud2", true));
    grid_display_->setFixedFrame("world");
    grid_display_->subProp("Cell Size")->setValue(0.15);
    grid_display_->subProp("Alpha")->setValue(0.5);
    cloud_points_display_->setFixedFrame("world");
    cloud_points_display_->subProp("Topic")->setValue(cloud_points_topic_);
    cloud_points_display_->subProp("Size (Pixels)")->setValue(size);
    cloud_points_display_->subProp("Style")->setValue("Points");
    cloud_points_display_->subProp("Color Transformer")->setValue("AxisColor");
    cloud_points_display_->subProp("Axis")->setValue("Z");
    connect(spin_point_size_.get(), QOverload<int>::of(&QSpinBox::valueChanged),
            [&](const int &value) {
              cloud_points_display_->subProp("Size (Pixels)")->setValue(value);
            });
    view_control_.reset(rviz_render_panel_->getViewController());
    // view_manager_.reset(rviz_visualization_manager_->getViewManager());
    // view_manager_->setCurrentViewControllerType("rviz/Orbit");
    view_control_->subProp("Distance")->setValue(1.08);
    view_control_->subProp("Yaw")->setValue(2);
    view_control_->subProp("Pitch")->setValue(0.74);
    view_control_->subProp("Focal Point")->subProp("X")->setValue(0.08);
    view_control_->subProp("Focal Point")->subProp("Y")->setValue(0.02);
    view_control_->subProp("Focal Point")->subProp("Z")->setValue(0.01);

    ROS_WARN_STREAM(cloud_points_topic_.toStdString());
    // QPointer<QHBoxLayout> z_height_lay_out = new QHBoxLayout();
    // QPointer<QLabel> lbl = new QLabel(u8"z轴高度差:");
    // spin_height_delta_ = QSharedPointer<QDoubleSpinBox>(new
    // QDoubleSpinBox()); spin_height_delta_->setReadOnly(true);
    // QPointer<QLabel> lbl_0 = new QLabel(u8"  x:");
    // spin_rot_x_ = QSharedPointer<QDoubleSpinBox>(new QDoubleSpinBox());
    // spin_rot_x_->setReadOnly(true);
    // QPointer<QLabel> lbl_1 = new QLabel(u8"  y:");
    // spin_rot_y_ = QSharedPointer<QDoubleSpinBox>(new QDoubleSpinBox());
    // spin_rot_y_->setReadOnly(true);
    // QPointer<QLabel> lbl_2 = new QLabel(u8"  z:");
    // spin_rot_z_ = QSharedPointer<QDoubleSpinBox>(new QDoubleSpinBox());
    // spin_rot_z_->setReadOnly(true);
    // QPointer<QLabel> lbl_3 = new QLabel(u8"  w:");
    // spin_rot_w_ = QSharedPointer<QDoubleSpinBox>(new QDoubleSpinBox());
    // spin_rot_w_->setReadOnly(true);
    // QPointer<QLabel> lbl_4 = new QLabel(u8"  平面度公差:");
    // lbl_plane_tolerance_ = QSharedPointer<QDoubleSpinBox>(new
    // QDoubleSpinBox()); lbl_plane_tolerance_->setReadOnly(true);
    // z_height_lay_out->addWidget(lbl_4);
    // z_height_lay_out->addWidget(lbl_plane_tolerance_.get());
    // z_height_lay_out->addWidget(lbl);
    // z_height_lay_out->addWidget(spin_height_delta_.get());
    // z_height_lay_out->addWidget(lbl_0);
    // z_height_lay_out->addWidget(spin_rot_x_.get());
    // z_height_lay_out->addWidget(lbl_1);
    // z_height_lay_out->addWidget(spin_rot_y_.get());
    // z_height_lay_out->addWidget(lbl_2);
    // z_height_lay_out->addWidget(spin_rot_z_.get());
    // z_height_lay_out->addWidget(lbl_3);
    // z_height_lay_out->addWidget(spin_rot_w_.get());
    // main_layout->addLayout(z_height_lay_out);
    QPointer<QHBoxLayout> param_lay_out = new QHBoxLayout();
    QPointer<QLabel> lbl_4 = new QLabel(u8"平面度公差:");
    lbl_4->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    lbl_plane_tolerance_ = QSharedPointer<QLabel>(new QLabel());
    lbl_plane_tolerance_->setNum(0.00f);
    // lbl_plane_tolerance_->setReadOnly(true);
    QPointer<QHBoxLayout> tolerance_lay_out = new QHBoxLayout();
    tolerance_lay_out->addWidget(lbl_4);
    tolerance_lay_out->addWidget(lbl_plane_tolerance_.get());
    lbl_detect_count_ = QSharedPointer<QLabel>(new QLabel());
    // lbl_detect_count_->setMaximum(std::numeric_limits<int>::max());
    // lbl_detect_count_->setReadOnly(true);
    lbl_detect_count_->setNum(static_cast<int>(detect_count_));
    QPointer<QLabel> lbl_5 = new QLabel(u8"检测次数:");
    lbl_5->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    QPointer<QHBoxLayout> h_lay = new QHBoxLayout();
    h_lay->addWidget(lbl_5);
    h_lay->addWidget(lbl_detect_count_.get());
    param_lay_out->addLayout(tolerance_lay_out);
    param_lay_out->addLayout(h_lay);
    QPointer<QLabel> lbl_6 = new QLabel(u8"最大平面度公差:");
    lbl_6->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    QPointer<QHBoxLayout> max_plane_tolerance_lay = new QHBoxLayout();
    lbl_max_plane_tolerance_ = QSharedPointer<QLabel>(new QLabel());
    // lbl_max_plane_tolerance_->setReadOnly(true);
    lbl_max_plane_tolerance_->setNum(0.00f);
    lbl_stamp_ = QSharedPointer<QLabel>(new QLabel());
    lbl_stamp_->setText(u8"暂无数据");
    max_plane_tolerance_lay->addWidget(lbl_6);
    max_plane_tolerance_lay->addWidget(lbl_max_plane_tolerance_.get());
    max_plane_tolerance_lay->addWidget(lbl_stamp_.get());
    param_lay_out->addLayout(max_plane_tolerance_lay);
    main_layout->addLayout(param_lay_out);
    text_edit_ = QSharedPointer<QTextEdit>(new QTextEdit());
    text_edit_->setMinimumHeight(150);
    text_edit_->setMaximumHeight(300);
    main_layout->addWidget(text_edit_.get());
  }
signals:
  void sender_setCurrentTaskFolder();

private:
  ros::NodeHandle nh_;
  flatness_detection::Param &param_;
  QString cloud_points_topic_, plane_tolerance_topic_, height_R_topic_;
  boost::shared_ptr<rviz::Display> grid_display_, cloud_points_display_;
  boost::shared_ptr<rviz::RenderPanel> rviz_render_panel_;
  boost::shared_ptr<rviz::VisualizationManager> rviz_visualization_manager_;
  boost::shared_ptr<rviz::ViewController> view_control_;
  boost::shared_ptr<rviz::ViewManager> view_manager_;
  QSharedPointer<QPushButton> btn_log_, btn_open_robot_;
  QSharedPointer<LogOutWidget> log_out_widget_;
  QSharedPointer<OutputWidget> output_widget_;
  QSharedPointer<QDoubleSpinBox> spin_height_delta_, spin_rot_x_, spin_rot_y_,
      spin_rot_z_, spin_rot_w_;
  QSharedPointer<QLabel> lbl_plane_tolerance_, lbl_max_plane_tolerance_,
      lbl_detect_count_;
  QSharedPointer<QSpinBox> spin_point_size_;
  QSharedPointer<QTextEdit> text_edit_;
  QSharedPointer<QLineEdit> line_edit_;
  QSharedPointer<QLabel> lbl_stamp_;
  ros::Subscriber sub_plane_tolerance_, sub_height_delta_R_, sub_cloud_points_,
      sub_web_btn_msgs_;
  ros::Publisher pub_cam_calibar_msgs_;
  boost::shared_ptr<rviz_visual_tools::RvizVisualTools> rviz_visual_tools_;
  QProcess process_;
  uint64_t detect_count_;
  std::map<double, QString> detect_time_map_;
  QFile file_terminal_;
  QSharedPointer<QTimer> timer_update_terminal_info_, gui_update_timer;
  std::mutex max_plane_tolerance_mutex_;
  std::pair<double, QString> max_plane_tolerance_;
  std::vector<std::pair<std::string, std::string>> item_;
  uint64_t report_index_, get_num_{0};
  json j_, json_;
  std::string json_path_, start_report_time_, path_, folder_;
  QString current_task_image_folder_;
  bool is_set_task_folder_;
  double flat_max_;

  JsonToCsvConverter converter;
};

int main(int argc, char **argv) {
  if (!ros::isInitialized()) {
    ros::init(argc, argv, "main_window", ros::init_options::AnonymousName);
  }
  QApplication app(argc, argv);
  boost::shared_ptr<FlatDetectionWidegt> flat_widget =
      boost::make_shared<FlatDetectionWidegt>();
  flat_widget->show();
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //  ros::spin();

  return app.exec();
}
#include "main_window.moc"