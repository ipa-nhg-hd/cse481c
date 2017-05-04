#include "pcl/point_cloud.h"
#include "pcl/common/common.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <limits.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

void GetMinMax(PointCloudC::Ptr input, double* min_x, double* max_x) {
  *min_x = std::numeric_limits<double>::max();
  *max_x = std::numeric_limits<double>::min();
  for (size_t i = 0; i < input->size(); ++i) {
    const PointC& pt = input->at(i);
    if (pt.x < *min_x) {
      *min_x = pt.x;
    }
    if (pt.x > *max_x) {
      *max_x = pt.x;
    }
  }
}

class Cropper {
 public:
  Cropper(const ros::Publisher& pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher pub_;
};

Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {}

void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param("crop_min_x", min_x, 0.3);
  ros::param::param("crop_min_y", min_y, -1.0);
  ros::param::param("crop_min_z", min_z, 0.5);
  ros::param::param("crop_max_x", max_x, 0.9);
  ros::param::param("crop_max_y", max_y, 1.0);
  ros::param::param("crop_max_z", max_z, 1.5);
  Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
  Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud);
  crop.setMin(min_pt);
  crop.setMax(max_pt);

  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  crop.filter(*cropped_cloud);

  PointC min_pcl;
  PointC max_pcl;
  pcl::getMinMax3D<PointC>(*cropped_cloud, min_pcl, max_pcl);
  ROS_INFO("min: %f, max: %f", min_pcl.x, max_pcl.x);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cropped_cloud, msg_out);
  pub_.publish(msg_out);
}

class Downsampler {
 public:
  Downsampler(const ros::Publisher& pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher pub_;
};

Downsampler::Downsampler(const ros::Publisher& pub) : pub_(pub) {}

void Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  PointCloudC::Ptr downsampled_cloud(new PointCloudC());
  pcl::VoxelGrid<PointC> vox;
  vox.setInputCloud(cloud);
  double voxel_size;
  ros::param::param("voxel_size", voxel_size, 0.01);
  vox.setLeafSize(voxel_size, voxel_size, voxel_size);
  vox.filter(*downsampled_cloud);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*downsampled_cloud, msg_out);
  pub_.publish(msg_out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  ros::Publisher downsampled_pub =
      nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
  Cropper cropper(crop_pub);
  Downsampler downsampler(downsampled_pub);
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &Cropper::Callback, &cropper);
  ros::Subscriber cropped_sub =
      nh.subscribe("cropped_cloud", 1, &Downsampler::Callback, &downsampler);
  ros::spin();
  return 0;
}
