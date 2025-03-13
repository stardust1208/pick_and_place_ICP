#include <ros/ros.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

// preprocessing raw point cloud (voxel downsampling and crop box)

class Process {
private:
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/point", 1, &Process::callback, this);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/pre_points", 1);
    double leaf_size; // VoxelGrid leaf size

public:
    Process() {
        nh.param("leaf_size", leaf_size, 0.005); // Default leaf size
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& input) {
        // Create PCL point clouds with color information
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_downsample(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_crop(new pcl::PointCloud<pcl::PointXYZRGB>());
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::fromROSMsg(*input, *pcl_input);
        // Apply VoxelGrid filter
        pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(pcl_input);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*pcl_downsample);
        // Crop box
        pcl::CropBox<pcl::PointXYZRGB> crop_filter;

        crop_filter.setMin(Eigen::Vector4f(-0.18, -0.12, 0.38, 1.0));
        crop_filter.setMax(Eigen::Vector4f(0.20, 0.15, 0.44, 1.0));
        //***********************************************
        // crop_filter.setMin(Eigen::Vector4f(-1.0, -1.0, -1.0, 1.0));
        // crop_filter.setMax(Eigen::Vector4f(1.0, 1.0, 1.0, 1.0));


        crop_filter.setInputCloud(pcl_downsample);
        //***********************************************
        // crop_filter.setInputCloud(pcl_input);

        crop_filter.filter(*pcl_crop);
        // Convert filtered PCL PointCloud back to ROS PointCloud2
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*pcl_crop, output);
        // Preserve the frame ID
        output.header = input->header;
        // Publish downsampled point cloud
        pub.publish(output);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pre_pointcloud");
    Process process;
    ros::spin();
    return 0;
}
