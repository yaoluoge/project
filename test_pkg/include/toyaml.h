#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <ros/package.h>
#include <Eigen/Dense>

void write_yaml_file(const Eigen::Matrix3d &R, const double &T, const std::string &str){
    
    YAML::Node node = YAML::LoadFile(ros::package::getPath("flatness_detect")+"/config/test1.yaml");

    int k = 0;

    node["%s", str.c_str()]["extrinsic_T"][2] = T;

    for (int i = 0; i < R.rows(); i++){
        for (int j = 0; j < R.cols(); j++){
            node["%s", str.c_str()]["extrinsic_R"][k] = R(i, j);
            k++;
        }
    }

    std::ofstream fout(ros::package::getPath("flatness_detect")+"/config/test1.yaml");
    fout << node;
    
    fout.close();
}