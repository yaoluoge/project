#include <chrono>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/convolution_3d.h>
#include <Eigen/Eigen>

void LSFP(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, Eigen::Vector3d &res, double &height_)
{
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Matrix<double, 3, 1> B = Eigen::Matrix<double, 3, 1>::Zero();

    A(2, 2) = msg->width;
    // printf("%d    %d\n", msg->width, msg->points.size());
    for (auto p : msg->points)
    {
        A(0, 0) += p.x * p.x;
        A(0, 1) += p.x * p.y;
        A(0, 2) += p.x;
        A(1, 0) += p.x * p.y;
        A(1, 1) += p.y * p.y;
        A(1, 2) += p.y;
        A(2, 0) += p.x;
        A(2, 1) += p.y;

        B(0, 0) += p.x * p.z;
        B(1, 0) += p.y * p.z;
        B(2, 0) += p.z;
    }
    Eigen::Matrix3d A_inv = A.inverse();
    
    Eigen::Vector3d abc = A_inv * B;
    res[0] = abc[0];
    res[1] = abc[1];
    res[2] = -1;
    res.normalize();

    height_ = abc[2];
    // printf("    %f\n", height_);
}
