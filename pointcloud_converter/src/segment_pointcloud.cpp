#include <ros/ros.h>
#include <msg_and_srv/PointCloud2Set.h>
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
    ros::Subscriber sub = nh.subscribe("/camera/pre_points", 1, &Process::callback, this);
    ros::Publisher pub = nh.advertise<msg_and_srv::PointCloud2Set>("/camera/obj_points", 1);

public:
    Process() {}

    void callback(const sensor_msgs::PointCloud2::ConstPtr& input) {
        // Convert ROS PointCloud2 to PCL PointCloud<pcl::PointXYZRGB>
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*input, *pcl_input);
        // KdTree object for search
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        tree->setInputCloud(pcl_input);
        // Perform Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.012);
        ec.setMinClusterSize(200);
        ec.setMaxClusterSize(10000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(pcl_input);
        ec.extract(cluster_indices);
        // Create output PointCloud2Set message
        msg_and_srv::PointCloud2Set output_msg;
        output_msg.header = input->header;
        // Process each cluster
        int cluster_id = 0;
        for (const auto& cluster : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
            // Extract points belonging to the cluster
            for (const auto& idx : cluster.indices) {
                cloud_cluster->push_back((*pcl_input)[idx]);
            }
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            // Convert PCL cluster to ROS PointCloud2
            sensor_msgs::PointCloud2 ros_cluster;
            pcl::toROSMsg(*cloud_cluster, ros_cluster);
            ros_cluster.header.frame_id = input->header.frame_id; // Maintain original frame ID
            // Add to PointCloud2Set message
            output_msg.pointcloud2.push_back(ros_cluster);
            cluster_id++;
        }
        output_msg.n_clusters = cluster_id;
        // Publish the clustered PointCloud2Set message
        pub.publish(output_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "segment_pointcloud");
    Process process;
    ros::spin();
    return 0;
}
