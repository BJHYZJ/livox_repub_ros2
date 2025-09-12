#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Point type definition with timestamp for deskewing
struct PointXYZIT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
)

typedef PointXYZIT PointType;

class LivoxRepub : public rclcpp::Node {
public:
    LivoxRepub() : Node("livox_repub") {
        // Initialize subscriber and publisher
        sub_livox_msg_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", 10,
                std::bind(&LivoxRepub::LivoxMsgCbk, this, std::placeholders::_1));
        pub_pcl_out_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                "/livox/points", 10);
    }

private:
    void LivoxMsgCbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received first LiDAR message");

        pcl::PointCloud<PointType>::Ptr pcl_in(new pcl::PointCloud<PointType>());

        for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
            PointType pt;
            pt.x = livox_msg->points[i].x;
            pt.y = livox_msg->points[i].y;
            pt.z = livox_msg->points[i].z;

            // Calculate intensity combining line number and reflectivity
            float s = livox_msg->points[i].offset_time / (float) livox_msg->points.back().offset_time;
            pt.intensity = livox_msg->points[i].line + livox_msg->points[i].reflectivity / 10000.0;

            // Set timestamp for deskewing (normalized to 0-1 range)
            pt.time = s;

            pcl_in->push_back(pt);
        }

        // Convert PointCloud to ROS message
        sensor_msgs::msg::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*pcl_in, pcl_ros_msg);

        // Set timestamp based on LiDAR message
        // rclcpp::Time ros_time(livox_msg->timebase);
        // pcl_ros_msg.header.stamp = ros_time;
        // Convert timebase (nanoseconds) to ROS time
        pcl_ros_msg.header.stamp = this->get_clock()->now();

        pcl_ros_msg.header.frame_id = livox_msg->header.frame_id;

        // Publish the converted PointCloud
        pub_pcl_out_->publish(pcl_ros_msg);
    }

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_msg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_out_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LivoxRepub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}