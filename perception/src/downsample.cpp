#include "perception/downsample.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception
{

Downsampler::Downsampler(const ros::Publisher &pub) : pub_(pub) {}

void Downsampler::Callback(const sensor_msgs::PointCloud2 &msg)
{
    PointCloudC::Ptr downsampled_cloud(new PointCloudC());
    pcl::fromROSMsg(msg, *downsampled_cloud);
    pcl::VoxelGrid<PointC> vox;
    double voxel_size;
    ros::param::param("voxel_size", voxel_size, 0.01);
    vox.setInputCloud(downsampled_cloud);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*downsampled_cloud);
    ROS_INFO("Downsampled to %ld points", downsampled_cloud->size());

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*downsampled_cloud, msg_out);
    pub_.publish(msg_out);
}
} // namespace perception