// TODO: add includes, etc.
#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception
{
    
Cropper::Cropper() {}

void Cropper::Callback(const sensor_msgs::PointCloud2 &msg)
{
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud);
    ROS_INFO("Got point cloud with %ld points", cloud->size());

    PointCloudC::Ptr cropped_cloud(new PointCloudC());
    Eigen::Vector4f min_pt(0.3, -1, 0.5, 1);
    Eigen::Vector4f max_pt(0.9, 1, 1.5, 1);
    pcl::CropBox<PointC> crop;
    crop.setInputCloud(cloud);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*cropped_cloud);
    ROS_INFO("Cropped to %ld points", cropped_cloud->size());
}
} // namespace perception