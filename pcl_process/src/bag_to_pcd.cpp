#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

using namespace std::chrono_literals;
pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud_(new pcl::PointCloud<pcl::PointXYZ>);

class BagToPcdConverter : public rclcpp::Node
{
public:
    BagToPcdConverter() : Node("bag_to_pcd_converter")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar_3GGDJ2Q00100411", 10, std::bind(&BagToPcdConverter::callback, this, std::placeholders::_1));
        
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
        

        *combined_cloud_ += *cloud;

        RCLCPP_INFO(this->get_logger(), "PCD file saved.");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BagToPcdConverter>();
    rclcpp::spin(node);
    if(!rclcpp::ok()){
        // Save point cloud to PCD file
        pcl::PCDWriter writer;
        writer.write("output.pcd", *combined_cloud_);
    }
    rclcpp::shutdown();
    return 0;
}