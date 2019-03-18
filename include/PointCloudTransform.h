
#ifndef POINTCLOUDTRANSFORM_H
#define POINTCLOUDTRANSFORM_H

#include <octomap/octomap.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

class PointCloudTransform
{
public:
    PointCloudTransform(){};
    PointCloudTransform(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalMap);
    void SetPointerPointcloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalMap);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transform();

protected:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud;

};

#endif // POINTCLOUDTRANSFORM_H