#include "main.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/conditional_removal.h>
#include <chrono>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <pcl/filters/extract_indices.h>
#include <flatness_detect/planeToleranceMsg.h>
#include <std_msgs/Float64.h>
// #include "global_emitter.hpp"

ros::Publisher pub_plane_tolerance;

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

double flatness_tolerance_band;
int COUNT = 0;
double TOLERANCE = 0.0;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam1;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam2;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam3;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam4;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fusion;

// rviz_visual_tools::RvizVisualToolsPtr visual_markers;

enum rotation
{
    cam1_w,
    cam2_w,
    cam3_w,
    cam4_w
};

Eigen::Matrix3d R_1_w, R_2_w, R_3_w, R_4_w;
Eigen::Vector3d T_1_w, T_2_w, T_3_w, T_4_w;

std::vector<double> extrinT_1_w(3, 0.0), extrinT_2_w(3, 0.0), extrinT_3_w(3, 0.0), extrinT_4_w(3, 0.0);
std::vector<double> extrinR_1_w(9, 0.0), extrinR_2_w(9, 0.0), extrinR_3_w(9, 0.0), extrinR_4_w(9, 0.0);

bool cam1tow = false, cam2tow = false, cam3tow = false, cam4tow = false;

bool judge_fun(pcl::PointCloud<pcl::PointXYZ> &msg, Eigen::VectorXf &pra_plane)
{
    // pcl 默认单位 m
    double flat = ransac_distancethreshold_;
    int num = msg.size();
    pcl::PointCloud<pcl::PointXYZ> msg_copy = msg;

    double A = pra_plane[0] / pra_plane[2];
    double B = pra_plane[1] / pra_plane[2];
    double D1 = pra_plane[3] / pra_plane[2];
    double dis = sqrt(A * A + B * B + 1);

    for (double i = flat; i < 0.004; i += 0.0001)
    {

        for (double j = -flat; j > -0.004; j -= 0.0001)
        {
            int count = 0;

            double D2 = D1 + dis * i;
            double D3 = D1 + dis * j;
            for (int k = 0; k < num; k++)
            {
                double homo = msg.points[k].x * A + msg.points[k].y * B + msg.points[k].z;
                if (homo + D2 > 0 && homo + D3 < 0)
                    count++;
            }
            if (count >= num * .996)
            // if(1)
            {
                if (COUNT >= mean_step)
                {

                    printf("平面度公差带为 %.2fmm!    \n\n", TOLERANCE / COUNT * 1000);
                    COUNT = 0;
                    TOLERANCE = 0;
                }
                else
                {
                    COUNT++;
                    TOLERANCE += i - j;
                }

                return true;
            }
        }
    }

    printf("未通过平面平整度检测,公差大于40mm!     \n\n");
    return false;
}

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

void cloud_kdtree(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, Eigen::Matrix3d &R, Eigen::Vector3d &T)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(msg);
    omp_set_num_threads(4);
    //  omp parallel num_threads(4);

#pragma omp parallel for
    for (size_t i = 0; i < msg->points.size(); i++)
    {
        if (msg->points[i].z == 0)
            continue;

        // Eigen::Vector3d p_body(msg->points[i].x, msg->points[i].y, msg->points[i].z);
        // Eigen::Vector3d p_global(R * p_body + T);
        // pcl::PointXYZ point_;
        // point_.x = p_global(0);
        // point_.y = p_global(1);
        // point_.z = p_global(2);

        int k = kdtree_neighbor_num_;
        std::vector<int> pointIdxNSearch(k);
        std::vector<float> pointNKNSquareDistance(k);
        if (kdtree.nearestKSearch(msg->points[i], k, pointIdxNSearch, pointNKNSquareDistance) > 0)
        {

            // 根据紧邻点判定
            int count = 0;
            for (int j = 1; j < k; j++)
            {
                double disz = msg->points[i].z - msg->points[pointIdxNSearch[j]].z;
                // double dis = disz / pointNKNSquareDistance[j];
                // double grad =  sqrt(dis2 - disz * disz);
                // abs(disz) /
                //  限定距离
                if (abs(disz) > kdtree_filter_dis_)
                    count++;
            }
            if (count > kdtree_filter_num_)
                continue;

            Eigen::Vector3d p_body(msg->points[i].x, msg->points[i].y, msg->points[i].z);
            Eigen::Vector3d p_global(R * p_body + T);
            pcl::PointXYZ point_;
            point_.x = p_global(0);
            point_.y = p_global(1);
            point_.z = p_global(2);
#pragma omp critical
            {
                cloud_fusion->points.push_back(point_);
                // cloud_cam2->push_back(msg->at(i));
                // cloud_cam2->width ,cloud_cam2->points.size()
            }
        }
    }
    cloud_fusion->resize(cloud_fusion->points.size());

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used1 = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    // std::cout << "cloud_kdtree count =  " << cloud->points.size();
    std::cout << "   kdtree time cost = " << time_used1.count() << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr statistic_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cf(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_(new pcl::PointCloud<pcl::PointXYZ>);
    // 统计滤波降噪
    for (auto p : msg->points)
    {
        if (p.z == 0)
            continue;
        msg_cf->points.push_back(p);
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(msg_cf);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.setNegative(false);
    sor.filter(*cloud_filter_);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used1 = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    std::cout << "statistic_filter count =  " << cloud_filter_->points.size();
    std::cout << "   statistic time cost = " << time_used1.count() << std::endl;
    return cloud_filter_;
}

void cloud_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    // std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    if (msg->width < cloud_filtered_width_)
        return;
    pcl::PCLPointCloud2 *cloud2 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2 cloud_filtered;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    pcl::toPCLPointCloud2(*msg, *cloud2);
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);

    double size = 0.0;

    // 体素滤波
    // if (cloud2->width > cloud_filtered_width_)
    // {
    do
    {
        size += 0.0005;
        sor.setInputCloud(cloudPtr);
        sor.setLeafSize(size, size, size);
        sor.filter(cloud_filtered);
    } while (cloud_filtered.width > cloud_filtered_width_);
    // }

    // while (cloud_filtered.width > cloud_filtered_width_){
    //     size += 0.0005;
    //     sor.setInputCloud(cloudPtr);
    //     sor.setLeafSize(size, size, size);
    //     sor.filter(cloud_filtered);
    // }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud_filtered, *msg);

    // std::cout << "cloud_filter count =  " << cloud->points.size() << "    leafsize = " << size;

    // std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    // std::chrono::duration<double> time_used2 = std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3);
    // std::cout << "   filter time cost = " << time_used2.count() << std::endl;
    // return cloud;
}

// 最小包围盒判别平面平整度
void bounding_box(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    flatness_tolerance_band = 0;
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moie;
    moie.setInputCloud(msg);
    moie.compute();
    pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    moie.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    double dx = max_point_OBB.x - min_point_OBB.x;
    double dy = max_point_OBB.y - min_point_OBB.y;
    double dz = max_point_OBB.z - min_point_OBB.z;
    // std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
    // ROS_INFO("cluster depth: %f, width: %f, height: %f", dx, dy, dz);
    flatness_tolerance_band = dx <= dy ? dx : dy;
    flatness_tolerance_band = flatness_tolerance_band <= dz ? flatness_tolerance_band : dz;

    COUNT++;
    TOLERANCE += flatness_tolerance_band;

    // std::chrono::duration<double> time_used4 = std::chrono::duration_cast<std::chrono::duration<double>>(t8 - t5);
    // std::cout << "    MBB time cost = " << time_used4.count() << std::endl << std::endl;

    Eigen::Isometry3d OBB_pose = Eigen::Isometry3d::Identity();
    OBB_pose.translation() << position_OBB.x, position_OBB.y, position_OBB.z;
    OBB_pose.rotate(rotational_matrix_OBB.cast<double>());
    Eigen::Quaterniond q(OBB_pose.rotation());

    // visual_markers->deleteAllMarkers();
    // visual_markers->publishWireframeCuboid(OBB_pose, dx, dy, dz);
    // visual_markers->trigger();

    sensor_msgs::PointCloud2 fit_input;
    pcl::toROSMsg(*msg, fit_input);
    // fit_input.header.frame_id = header_.frame_id;
    fit_input.header.frame_id = "world";
    pub_fit_input.publish(fit_input);
}

void fit_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_interior(new pcl::PointCloud<pcl::PointXYZ>);
    // std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromPCLPointCloud2(msg, *cloud_filtered);

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(msg));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);

    ransac.setDistanceThreshold(ransac_distancethreshold_);
    ransac.setMaxIterations(maxiterations_);
    ransac.setProbability(probability_);
    ransac.computeModel();
    // copy 内点 cloud_interior
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_of_distance(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> inliers;
    ransac.getInliers(inliers);

    pcl::copyPointCloud<pcl::PointXYZ>(*msg, inliers, *cloud_interior);

    // get 平面参数
    Eigen::VectorXf coefficient;
    ransac.getModelCoefficients(coefficient);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(inliers);
    extract.setInputCloud(msg);
    extract.setIndices(index_ptr);
    extract.setNegative(true);
    extract.filter(*out_of_distance);
    // std::cout << "平面方程为： \n"
    //           << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + " << coefficient[3] << "= 0" << std::endl;

    // std::cout << cloud_filtered->points.size() << std::endl;

    // std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
    // std::chrono::duration<double> time_used3 = std::chrono::duration_cast<std::chrono::duration<double>>(t6 - t5);
    // ROS_INFO("fit time cost = %f", time_used3.count());
    // std::cout << "fit time cost = " << time_used3.count() ;
    // std::cout <<  "     ransac.iterations_ = " << ransac.iterations_ << std::endl;

    // 计算不符合规定的点云数量
    // int sum = 0;

    // for (size_t i = 0; i < msg->points.size(); i++)
    // {
    //     double distance = msg->points[i].x * coefficient[0] + msg->points[i].y * coefficient[1] + msg->points[i].z * coefficient[2] + coefficient[3];
    //     distance /= sqrt(coefficient[0] * coefficient[0] + coefficient[1] * coefficient[1] + coefficient[2] * coefficient[2]);
    //     if (distance > distance_threshold_)
    //     {
    //         out_of_distance->points.push_back(msg->points[i]);
    //         sum++;
    //     }
    // }

    // std::cout
    //     << "sum = " << sum;
    // std::cout << "   size = " << msg->points.size() << "    radio = " << (double)sum / msg->points.size() << std::endl;

    // 判决函数
    // std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
    if (cloud_interior->points.size() < 0.996 * msg->points.size())
        bool result = judge_fun(*msg, coefficient);
    else
        printf("\n pass detect, 平整度 < %.2f !\n", 2 * ransac_distancethreshold_);
    // std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
    // std::chrono::duration<double> time_used4 = std::chrono::duration_cast<std::chrono::duration<double>>(t8 - t7);
    // std::cout << "judge time cost = " << time_used4.count() << std::endl << std::endl;

    sensor_msgs::PointCloud2 fit_input;
    sensor_msgs::PointCloud2 plane_inliers;
    sensor_msgs::PointCloud2 point_out_of_distance;
    pcl::toROSMsg(*cloud_interior, plane_inliers);
    pcl::toROSMsg(*msg, fit_input);
    pcl::toROSMsg(*out_of_distance, point_out_of_distance);
    fit_input.header.frame_id = "world"; // camera_depth_optical_frame
    plane_inliers.header.frame_id = "world";
    point_out_of_distance.header.frame_id = "world";
    pub_fit_input.publish(fit_input);
    pub_plane_inliers.publish(plane_inliers);
    pub_point_out_of_distance.publish(point_out_of_distance);
}

void static_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud_1,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud_2,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud_3,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud_4)
{
    cloud_fusion.reset(new pcl::PointCloud<pcl::PointXYZ>);
    if (!point_cloud_1->empty())
        cloud_kdtree(point_cloud_1, R_1_w, T_1_w);
    if (!point_cloud_2->empty())
        cloud_kdtree(point_cloud_2, R_2_w, T_2_w);
    if (!point_cloud_3->empty())
        cloud_kdtree(point_cloud_3, R_3_w, T_3_w);
    if (!point_cloud_4->empty())
        cloud_kdtree(point_cloud_4, R_4_w, T_4_w);
    if (cloud_fusion->empty())
        return;

    // cloud_fusion.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // cloud_fusion = point_cloud_mid;

    // 体素滤波
    cloud_filter(cloud_fusion);

    // 判决函数
    if (using_MBB_)
        bounding_box(cloud_fusion);
    else
        fit_plane(cloud_fusion);
}

void imgCb(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, const rotation param)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (first_frame)
    {
        header_ = cloud_msg->header;
        header_.frame_id = "world";
        // visual_markers.reset(new rviz_visual_tools::RvizVisualTools(header_.frame_id, "/visual_markers"));
        // visual_markers->loadMarkerPub(false, true); // create publisher before waiting
        // visual_markers->deleteAllMarkers();
        // visual_markers->enableBatchPublishing(true);
        if (param == cam2_w)
            first_frame = false;
    }

    pcl::fromROSMsg(*cloud_msg, *cloud);
    // if (cloud->empty())
    //     return;
    if (param == cam2_w && cam1tow == false)
    {
        cloud_cam2.reset(new pcl::PointCloud<pcl::PointXYZ>);
        Conditional_filter(cloud, cloud_cam2);
        cam1tow = true;
    }
    else if (param == cam1_w && cam2tow == false)
    {
        cloud_cam1.reset(new pcl::PointCloud<pcl::PointXYZ>);
        Conditional_filter(cloud, cloud_cam1);
        cam2tow = true;
    }
    else if (param == cam3_w && cam3tow == false)
    {
        cloud_cam3.reset(new pcl::PointCloud<pcl::PointXYZ>);
        Conditional_filter(cloud, cloud_cam3);
        cam3tow = true;
    }
    else if (param == cam4_w && cam4tow == false)
    {
        cloud_cam4.reset(new pcl::PointCloud<pcl::PointXYZ>);
        Conditional_filter(cloud, cloud_cam4);
        cam4tow = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud");

    ros::NodeHandle nh("~");

    nh.param<int>("cloud_filtered_width", cloud_filtered_width_, 1200);
    nh.param<int>("kdtree_neighbor_num", kdtree_neighbor_num_, 21);
    nh.param<double>("kdtree_filter_dis", kdtree_filter_dis_, 0.0005);
    nh.param<int>("kdtree_filter_num", kdtree_filter_num_, 5);
    nh.param<double>("ransac_distancethreshold", ransac_distancethreshold_, 0.0003);
    nh.param<int>("ransac_maxiterations", maxiterations_, 5000);
    nh.param<double>("ransac_probability", probability_, 0.99);
    nh.param<double>("distance_threshold", distance_threshold_, 0.0015);
    nh.param<bool>("using_MBB", using_MBB_, 0.0015);
    nh.param<double>("x_GT", x_GT_, 0.0015);
    nh.param<double>("x_LT", x_LT_, 0.0015);
    nh.param<double>("y_GT", y_GT_, 0.0015);
    nh.param<double>("y_LT", y_LT_, 0.0015);
    nh.param<double>("z_GT", z_GT_, 0.0015);
    nh.param<double>("z_LT", z_LT_, 0.0015);
    nh.param<int>("mean_step", mean_step, 15);
    nh.param<std::vector<double>>("/D435i_1/depth/color/points/extrinsic_R", extrinR_1_w, std::vector<double>());
    nh.param<std::vector<double>>("/D435i_1/depth/color/points/extrinsic_T", extrinT_1_w, std::vector<double>());
    nh.param<std::vector<double>>("/D435i_2/depth/color/points/extrinsic_R", extrinR_2_w, std::vector<double>());
    nh.param<std::vector<double>>("/D435i_2/depth/color/points/extrinsic_T", extrinT_2_w, std::vector<double>());
    nh.param<std::vector<double>>("/D435i_3/depth/color/points/extrinsic_R", extrinR_3_w, std::vector<double>());
    nh.param<std::vector<double>>("/D435i_3/depth/color/points/extrinsic_T", extrinT_3_w, std::vector<double>());
    nh.param<std::vector<double>>("/D435i_4/depth/color/points/extrinsic_R", extrinR_4_w, std::vector<double>());
    nh.param<std::vector<double>>("/D435i_4/depth/color/points/extrinsic_T", extrinT_4_w, std::vector<double>());

    std::string image_topic_1 = nh.param<std::string>("cloud_topic_1", "/ascamera1/depth/points0");
    std::string image_topic_2 = nh.param<std::string>("cloud_topic_2", "/ascamera1/depth/points0");
    std::string image_topic_3 = nh.param<std::string>("cloud_topic_3", "/ascamera1/depth/points0");
    std::string image_topic_4 = nh.param<std::string>("cloud_topic_4", "/ascamera1/depth/points0");

    pub_plane_inliers = nh.advertise<pcl::PCLPointCloud2>("out_plane_inliers", 1000);
    pub_fit_input = nh.advertise<sensor_msgs::PointCloud2>("out_fit_input", 1000); // 显示：点云
    pub_point_out_of_distance = nh.advertise<pcl::PCLPointCloud2>("point_out_of_distance", 1000);

    pub_plane_tolerance = nh.advertise<flatness_detect::planeToleranceMsg>("plane_tolerance", 1, true);

    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2>(image_topic_1, 1, std::bind(imgCb, std::placeholders::_1, cam1_w));
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2>(image_topic_2, 1, std::bind(imgCb, std::placeholders::_1, cam2_w));
    ros::Subscriber sub3 = nh.subscribe<sensor_msgs::PointCloud2>(image_topic_3, 1, std::bind(imgCb, std::placeholders::_1, cam3_w));
    ros::Subscriber sub4 = nh.subscribe<sensor_msgs::PointCloud2>(image_topic_4, 1, std::bind(imgCb, std::placeholders::_1, cam4_w));

    R_1_w << MAT_FROM_ARRAY(extrinR_1_w);
    T_1_w << VEC_FROM_ARRAY(extrinT_1_w);
    R_2_w << MAT_FROM_ARRAY(extrinR_2_w);
    T_2_w << VEC_FROM_ARRAY(extrinT_2_w);
    R_3_w << MAT_FROM_ARRAY(extrinR_3_w);
    T_3_w << VEC_FROM_ARRAY(extrinT_3_w);
    R_4_w << MAT_FROM_ARRAY(extrinR_4_w);
    T_4_w << VEC_FROM_ARRAY(extrinT_4_w);

    // GlobalSignalsEmitter &global_emitter{GlobalSignalsEmitter::getInstance()};
    // bool first_time = true;
    while (ros::ok())
    {
        
        if (cam1tow && cam2tow && cam3tow && cam4tow)
        {
            std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
            static_transform(cloud_cam1, cloud_cam2, cloud_cam3, cloud_cam4);
            std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used4 = std::chrono::duration_cast<std::chrono::duration<double>>(t8 - t7);
            std::cout << "frequence" << 1 / time_used4.count() << std::endl;
            cam1tow = false;
            cam2tow = false;
            cam3tow = false;
            cam4tow = false;

            if (COUNT >= mean_step)
            {
                // printf("平面度公差带为 %.4fmm\n\n", TOLERANCE / COUNT * 1000); // 平面带公差：TOLERANCE / COUNT * 1000

                // flatness_detect::planeToleranceMsg flatness_tolerance_msg;
                // flatness_tolerance_msg.plane_tolerance = TOLERANCE / COUNT * 1000;
                // flatness_tolerance_msg.header.stamp = ros::Time::now();
                // pub_plane_tolerance.publish(flatness_tolerance_msg);

                // global_emitter.globalEmitSignals("平面度公差带: " + QString::number(TOLERANCE / COUNT * 1000, 'f', 4));

                // COUNT = 0;
                // TOLERANCE = 0;


                printf("平面度公差带为 %.4fmm\n\n", TOLERANCE / COUNT * 1000); // 平面带公差：TOLERANCE / COUNT * 1000

                flatness_detect::planeToleranceMsg flatness_tolerance_msg;
                flatness_tolerance_msg.plane_tolerance = TOLERANCE / COUNT * 1000;
                flatness_tolerance_msg.header.stamp=ros::Time::now();
                flatness_tolerance_msg.info_error="平面带公差："+std::to_string(TOLERANCE / COUNT * 1000) + "mm";
                pub_plane_tolerance.publish(flatness_tolerance_msg);

                // global_emitter.emitGlobalSignal("平面带公差："+QString::number(TOLERANCE / COUNT * 1000,'f',4),"");

                COUNT = 0;
                TOLERANCE = 0;
            }
        }
        usleep(100);
        ros::spinOnce();
    }
}