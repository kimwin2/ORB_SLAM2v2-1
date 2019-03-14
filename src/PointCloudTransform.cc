#include "PointCloudTransform.h"
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

PointCloudTransform::PointCloudTransform(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalMap){
    pcloud = globalMap;
}

void PointCloudTransform::SetPointerPointcloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalMap){
    pcloud = globalMap;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudTransform::transform(){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA> ()) ;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBA> ()) ;
    std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA>> tpoints;
    Eigen::Matrix4f transform_trans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_rot_x;
    Eigen::Matrix4f transform_rot_y;
    //Eigen::Matrix4f transform_rot_z = Eigen::Matrix4f::Identity();

    //translation matrix
    transform_trans(0,0) = 1		;transform_trans(0,1) = 0		;transform_trans(0,2) = 0;
    transform_trans(1,0) = 0		;transform_trans(1,1) = 0		;transform_trans(1,2) = 1;
    transform_trans(2,0) = 0		;transform_trans(2,1) = -1		;transform_trans(2,2) = 0;

    pcl::transformPointCloud (*pcloud, *transformed_cloud, transform_trans);

    tpoints = transformed_cloud->points;

    for(int i = 0; i < tpoints.size(); i++){
        if(tpoints.at(i).z > 0)
            temp->points.push_back(tpoints.at(i));
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold(0.01);
    seg.setMaxIterations (2000);
    seg.setInputCloud(temp);
    seg.segment(*inliers, *coefficients);


    cout << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << endl;

    double yzp = sqrt(1 - coefficients->values[0] * coefficients->values[0]);
    transform_rot_x << 1, 0, 0, 0,
                        0, (coefficients->values[2]/yzp), -(coefficients->values[1]/yzp), 0,
                        0, (coefficients->values[1]/yzp), (coefficients->values[2]/yzp), 0,
                        0, 0, 0, 1;
    double xzp = sqrt(1 - coefficients->values[1] * coefficients->values[1]);
    transform_rot_y << (coefficients->values[2]/xzp), 0, (coefficients->values[0]/xzp), 0,
                        0, 1, 0, 0,
                        -(coefficients->values[0]/xzp), 0, (coefficients->values[2]/xzp), 0,
                        0, 0, 0, 1;

    pcl::transformPointCloud(*pcloud, *transformed_cloud, transform_trans * transform_rot_x * transform_rot_y);

    return transformed_cloud;
}