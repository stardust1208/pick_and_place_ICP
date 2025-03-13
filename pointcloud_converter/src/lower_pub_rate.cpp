#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class DataProcessor {
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Rate rate_;
    sensor_msgs::PointCloud2 last_msg;
    bool new_data_available_;

public:
    DataProcessor() : rate_(10), new_data_available_(false) { // Publishing at 10 Hz
        sub = nh.subscribe("/camera/depth/color/points", 1, &DataProcessor::callback, this); // Subscribing at a sufficient queue size
        pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/point", 1);
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) { // Replace message type accordingly
        last_msg = *msg;
        new_data_available_ = true;
    }

    void spin() {
        while (ros::ok()) {
            ros::spinOnce(); // Process incoming messages

            // Publish data at 10 Hz
            if (new_data_available_) {
                pub.publish(last_msg);
                new_data_available_ = false; // Reset the flag
            }

            rate_.sleep(); // Maintain 10 Hz rate
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lower_pub_rate");
    DataProcessor processor;
    processor.spin();
    return 0;
}
