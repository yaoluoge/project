#include <chrono>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <omp.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <LSFP.h>
#include <thread>
#include <pcl/filters/conditional_removal.h>
#include <toyaml.h>
#include <flatness_detect/planeToleranceMsg.h>
#include <geometry_msgs/Quaternion.h>
// #include <Eigen/Dense>

ros::Publisher pub_height_delta_R;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target;
int max_iteration;
bool fitting_mathod_LSFP;
double x_GT_;
double x_LT_;
double y_GT_;
double y_LT_;
double z_GT_;
double z_LT_;

// bool registration_method_;
int count = 0;
double height_delta;
double sum_height_delta = 0;
Eigen::Matrix3d R;
Eigen::Quaterniond q_sum(0, 0, 0, 0);
Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity();
// 旋转矩阵 R_L2M, R_R2M
enum rotation
{
    SOURCE,
    TARGET
};

Eigen::Matrix4d R_L2M, R_M2M, R_R2M = Eigen::Matrix4d::Identity();

bool sub_tar = false, sub_sor = false;

void Conditional_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &res_msg)
{
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, y_GT_)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, y_LT_)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, x_GT_)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, x_LT_)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, z_GT_)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, z_LT_)));

    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(msg);
    condrem.filter(*res_msg);
}

void cloud_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_(new pcl::PointCloud<pcl::PointXYZ>);
    // 统计滤波降噪
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(msg);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.setNegative(false);
    sor.filter(*msg);

    // 高斯滤波平滑噪声
    if (false)
    {
        pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr Kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
        (*Kernel).setSigma(.0005);
        (*Kernel).setThresholdRelativeToSigma(.0004);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        (*kdtree).setInputCloud(msg);

        pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
        convolution.setKernel(*Kernel);
        convolution.setInputCloud(msg);
        convolution.setSearchMethod(kdtree);
        convolution.setRadiusSearch(.003);
        convolution.setNumberOfThreads(10);
        convolution.convolve(*msg);
    }
    // 体素滤波降采样
    // pcl::VoxelGrid<pcl::PointXYZ> voxel;
    // double size = 0;

    // 自适应体素滤波
    // do
    // {
    //     size += 0.002;
    //     voxel.setInputCloud(msg);
    //     voxel.setLeafSize(size, size, size);
    //     voxel.filter(*msg);
    // } while (msg->points.size() > 2000);

    // 遍历点云并进行外参变换
    // if (R == M2M)
    //     return msg;

    // Eigen::Matrix3d R_pose;
    // Eigen::Vector3d T_pose;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr point_pose(new pcl::PointCloud<pcl::PointXYZ>);
    // point_pose = msg;
    // R == L2M ? (T_pose = R_L2M.block<3, 1>(3, 0), R_pose = R_L2M.block<3, 3>(0, 0))
    //          : (T_pose = R_R2M.block<3, 1>(3, 0), R_pose = R_R2M.block<3, 3>(0, 0));

    //     omp_set_num_threads(10);
    // #pragma omp parallel for
    // for (auto p : msg->points)
    // {
    //     Eigen::Vector3d point(p.x, p.y, p.z);
    //     point = R_pose * point + T_pose;
    //     pcl::PointXYZ point_(point[0], point[1], point[2]);
    //     // #pragma omp critical
    //     point_pose->points.push_back(point_);
    // }
    // return point_pose;
}

// 配准方案2
// 拟合平面参数求解外参
// 求解平面法向量的 R，T 变换，
bool PC_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &source)
{
    Eigen::Vector3d res_source, res_target(0, 0, -1);
    double source_height, target_height = 0.0;
    // 最小二乘拟合平面

    if (fitting_mathod_LSFP)
    {
        LSFP(source, res_source, source_height);
    }
    else
    {
    }

    // RANSAC拟合平面
    height_delta = target_height - source_height;
    // std::cout << "height delta : " << target_height << "   " << source_height << std::endl;
    // std::cout << "res : " << res_source.matrix() << "   " << res_target.matrix() << std::endl;
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(res_source, res_target);

    count++;

    sum_height_delta += height_delta;
    height_delta = sum_height_delta / double(count);

    q_sum.w() += q.w();
    q_sum.x() += q.x();
    q_sum.y() += q.y();
    q_sum.z() += q.z();

    q.w() = q_sum.w() / count;
    q.x() = q_sum.x() / count;
    q.y() = q_sum.y() / count;
    q.z() = q_sum.z() / count;
    q.normalize();
    R = q.toRotationMatrix();

    // std::cout << "height delta : " << height_delta << std::endl;
    // std::cout << "Rotation : " << R.matrix() << std::endl;
    return true;
}

void pointCb(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    cloud_source.reset(new pcl::PointCloud<pcl::PointXYZ>);
    Conditional_filter(cloud, cloud_source);
    cloud_filter(cloud_source);

    std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
    PC_registration(cloud_source);
    std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used4 = std::chrono::duration_cast<std::chrono::duration<double>>(t8 - t7);

    printf("loading : %.1f%%\n", float(count) / float(max_iteration) * 100);
    printf("\33[A");
    printf("\33[K");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PEC");

    ros::NodeHandle nh("~");

    std_msgs::Header header_;

    // nh.param<bool>("registration_method", registration_method_, true);

    std::string cloud_topic = nh.param<std::string>("cloud_topic", "/D435i_3/depth/color/points");
    nh.param<double>("x_GT", x_GT_, 0.0015);
    nh.param<double>("x_LT", x_LT_, 0.0015);
    nh.param<double>("y_GT", y_GT_, 0.0015);
    nh.param<double>("y_LT", y_LT_, 0.0015);
    nh.param<double>("z_GT", z_GT_, 0.0015);
    nh.param<double>("z_LT", z_LT_, 0.0015);
    nh.param<int>("max_iteration", max_iteration, 15);
    nh.param<bool>("fitting_method_LSFP", fitting_mathod_LSFP, true);

    // ros::Publisher pub_plane_inliers;
    // ros::Publisher pub_fit_input;
    // ros::Publisher pub_point_out_of_distance;

    // pub_plane_inliers = nh.advertise<pcl::PCLPointCloud2>("out_plane_inliers", 1000);
    // pub_fit_input = nh.advertise<pcl::PCLPointCloud2>("out_fit_input", 1000);
    // pub_point_out_of_distance = nh.advertise<pcl::PCLPointCloud2>("point_out_of_distance", 1000);

    pub_height_delta_R = nh.advertise<flatness_detect::planeToleranceMsg>("height_delta_R", 1, true);

    const char *stringarray[] = {"/D435i_1/depth/color/points", "/D435i_2/depth/color/points", "/D435i_3/depth/color/points", "/D435i_4/depth/color/points"};

    for (auto &feat : stringarray)
    {
        cloud_topic = feat;
        ros::Subscriber sub_source = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 1, pointCb);
        while (count < max_iteration && ros::ok())
        {
            ros::spinOnce();
        }
        write_yaml_file(R, height_delta, cloud_topic); // 显示：height_delta（z的外参） R（旋转外参）**显示四个相机**

        flatness_detect::planeToleranceMsg height_delta_R_msg;
        height_delta_R_msg.height_delta = height_delta;
        Eigen::Quaterniond q{R};
        height_delta_R_msg.R.x = q.x();
        height_delta_R_msg.R.y = q.y();
        height_delta_R_msg.R.z = q.z();
        height_delta_R_msg.R.w = q.w();
        height_delta_R_msg.cam_topic_name = cloud_topic;
        pub_height_delta_R.publish(height_delta_R_msg);

        std::cout << cloud_topic << " : " << std::endl
                  << "height delta : " << height_delta << std::endl;
        std::cout << "Rotation : " << R.matrix() << std::endl
                  << std::endl;
        printf("%s\n", feat);
        count = 0;
        sum_height_delta = 0;
        q_sum = Eigen::Quaterniond(0, 0, 0, 0);
    }
    return 0;
}
