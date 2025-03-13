#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <msg_and_srv/PointCloud2Set.h>

class Process {
private:
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/first_cluster", 1);
    ros::Subscriber sub = nh.subscribe("/camera/obj_points", 1, &Process::callback, this);
public:
    Process() {}
    void callback(const msg_and_srv::PointCloud2Set::ConstPtr& msg) {
        if (msg->n_clusters > 0 && !msg->pointcloud2.empty()) {
            // Get the first cluster
            sensor_msgs::PointCloud2 first_cluster = msg->pointcloud2[0];
            pub.publish(first_cluster);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "first_points");
    Process process;
    ros::spin();
    return 0;
}
