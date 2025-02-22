#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <omp.h>
#include "geometry_msgs/Twist.h"
#include <vector>
// class VisualOdometry
// {
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//     typedef std::shared_ptr<VisualOdometry> Ptr;

/// constructor with config file
// VisualOdometry(const ros::NodeHandle &nh);

// void Run();

// public:
// PointCloudXYZI pl_full, pl_corn, pl_surf;
// ros::Publisher pub_full, pub_surf, pub_corn;

// private:
ros::Publisher vel_pub;

bool judge_fun(pcl::PointCloud<pcl::PointXYZ> &msg, Eigen::VectorXf &pra_plane);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_kdtree(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);

pcl::PointCloud<pcl::PointXYZ>::Ptr statistic_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(pcl::PointCloud<pcl::PointXYZ> &msg);

void pointCb(const sensor_msgs::PointCloud2::ConstPtr &msg);

void fit_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);

std_msgs::Header header_;

ros::Publisher pub_plane_inliers;
ros::Publisher pub_fit_input;
ros::Publisher pub_point_out_of_distance;

double ransac_distancethreshold_;
int maxiterations_;
double probability_;
double distance_threshold_;
int cloud_filtered_width_;
int kdtree_neighbor_num_;
double kdtree_filter_dis_;
int kdtree_filter_num_;
bool using_MBB_;
double x_GT_;
double x_LT_;
double y_GT_;
double y_LT_;
double z_GT_;
double z_LT_;
double flat_max_;
int mean_step, fixed_length_detection;
bool first_frame = true;
int signal1 = 0;

geometry_msgs::Twist vel_flat2control;
float wheel_odom = 0;
double delta_time = 0.0f;
bool first_wheel = true;
bool reset_odom_ = true;
// 第measure_num个检测任务
int measure_num = 0;
double detect_interval;
double slip_compensation;
// 第measure_num个任务的第detect_count个监测点
int detect_count = 0;
std::vector<double> defect_location_x, defect_location_y;
double wheel_time, wheel_time_last;

std::mutex mtx;

// };
