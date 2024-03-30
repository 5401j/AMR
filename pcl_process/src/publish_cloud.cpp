#include <iostream>
#include <memory>
#include <string>
#include <iomanip>
#include <vector>
#include <chrono>
#include <ctime>
#include <cmath>
#include <boost/filesystem.hpp>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

using namespace std;
pcl::PointCloud<pcl::PointXYZ>::Ptr horizon_1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr horizon_2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr mid70_1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr mid70_2(new pcl::PointCloud<pcl::PointXYZ>);

class CloudSubscriberPublisher : public rclcpp::Node
{
public:
    CloudSubscriberPublisher() : Node("cloud_subscriber_publisher")
    {
        counter=0;
        topic_head="/livox/lidar_";

        bd_code.push_back("3WEDH7600101861");
        bd_code.push_back("3GGDJ2Q00100411");
        bd_code.push_back("3WEDH7600101861");
        bd_code.push_back("3GGDJ2Q00100411");

        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("output_cloud", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&CloudSubscriberPublisher::timer_callback, this));
    }

private:
    void nothingCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        cloud_pub_->publish(*cloud_msg);
    }
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        cloud_pub_->publish(*cloud_msg);
        string device_id = string(cloud_sub_->get_topic_name()).substr(13);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        if(device_id=="3WEDH7600101861"){
            *horizon_1 += *cloud;
        }else if(device_id=="3GGDJ2Q00100411"){
            *mid70_1 += *cloud;
        }else if(device_id=="horizon2"){
            *horizon_2 += *cloud;
        }else if(device_id=="mid702"){
            *mid70_2 += *cloud;
        }
        
    }
    void timer_callback()
    {   
        cloud_sub_.reset();
        // nothing_sub_.reset();
        if(counter<bd_code.size()){
            std::cout<<"current topic is "<<counter<<std::endl;
            cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_head+bd_code[counter], 10, std::bind(&CloudSubscriberPublisher::cloudCallback, this, std::placeholders::_1));
            ++counter;
        }else{
            timer_->cancel();
            rclcpp::shutdown();
            // cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            //     topic_head+bd_code[0], 10, std::bind(&CloudSubscriberPublisher::nothingCallback, this, std::placeholders::_1));
        }

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr nothing_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    vector<string> bd_code;
    string topic_head;
    int counter;
};

void save_pc(string pc_name , pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr){
    pcl::PCDWriter writer;
    if(!pc_ptr->empty())
        writer.write(pc_name, *pc_ptr);
}

int main(int argc, char **argv)
{
    time_t now=time(nullptr);
    tm* t = localtime(&now);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%m%d%H%M%S", t);
    string folder=buffer;
    if (!boost::filesystem::exists(folder)) {
        boost::filesystem::create_directory(folder);
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudSubscriberPublisher>();
    rclcpp::spin(node);
    if(!rclcpp::ok()){
        // Save point cloud to PCD file
        save_pc(folder+"/mid70_1.pcd", mid70_1);
        save_pc(folder+"/horizon_1.pcd", horizon_1);
        save_pc(folder+"/mid70_2.pcd", mid70_2);
        save_pc(folder+"/horizon_2.pcd", horizon_2);
        cout<<"bye~~~"<<endl;
    }
    
    return 0;
}