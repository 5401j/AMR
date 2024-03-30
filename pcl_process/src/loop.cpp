#include <memory>
#include <string>
#include <iomanip>
#include <vector>
#include <chrono>
#include <time.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;
#define BOOST_BIND_NO_PLACEHOLDERS
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/random_sample.h>

#include <Eigen/Dense>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace Eigen;

class AMRcontroler : public rclcpp::Node
{
public:
    AMRcontroler()
        : Node("control_amr")
    {
        subscription_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "horizon", 1, std::bind(&AMRcontroler::topic_callback, this, _1));
        subscription_distance = this->create_subscription<std_msgs::msg::Float32>(
            "distance", 1, std::bind(&AMRcontroler::distance_callback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    Vector3f ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double th, Vector3f Coeff)
    {
        srand(time(NULL));
        // cout << "ransac start\n";

        size_t iteration, t = 0;
        std::vector<int> best_indexs;
        pcl::PointXYZ min, max;

        getMinMax3D(*cloud, min, max);
        // ransac迭代次數計算
        double param = pow(0.2, 3);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ransac(new pcl::PointCloud<pcl::PointXYZ>);
        iteration = log(1 - 0.99) / log(1 - param);
        vector<int> bestInlierIndices;
        // cout << iteration << endl;

        // XXXXXX
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_random3(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RandomSample<pcl::PointXYZ> random;
        random.setInputCloud(cloud);
        random.setSample(3);
        random.setSeed(rand());

        // iteration = 1;
        std::vector<int> indexs;
        indexs.reserve(cloud->size());
        while (t < iteration)
        {
            // 隨機選擇3個點
            indexs.clear();
            random.filter(*cloud_random3);

            double a, b, c; // ax^2+bx+c
            double x1_x3 = cloud_random3->points[0].x - cloud_random3->points[2].x;
            double x1_x2 = cloud_random3->points[0].x - cloud_random3->points[1].x;
            double x3_x2 = cloud_random3->points[2].x - cloud_random3->points[1].x;
            a = (cloud_random3->points[0].y - cloud_random3->points[2].y) / (x1_x3 * x3_x2) - (cloud_random3->points[0].y - cloud_random3->points[1].y) / (x1_x2 * x3_x2);
            b = (cloud_random3->points[0].y - cloud_random3->points[1].y) / x1_x2 - a * (cloud_random3->points[0].x + cloud_random3->points[1].x);
            c = cloud_random3->points[0].y - b * cloud_random3->points[0].x - a * cloud_random3->points[0].x * cloud_random3->points[0].x;

            // 參數a或b不符合預期的濾除
            if (abs(a) > 0.01 || abs(b) > 0.15)
            {
                t++;
                continue;
            }

            // ransac過程
            for (size_t i = 0; i < cloud->size(); i++)
            {
                auto yi = a * cloud->points[i].x * cloud->points[i].x + b * cloud->points[i].x + c - cloud->points[i].y;
                if (abs(yi) <= th)
                    indexs.push_back(i);
            }

            if (indexs.size() > best_indexs.size())
            {
                Coeff(0) = a;
                Coeff(1) = b;
                Coeff(2) = c;
                best_indexs = indexs;
            }
            t++;
        }

        // cout << "實際內點比例" << best_indexs.size() << ":" << cloud->size() << "\n";

        // pcl::copyPointCloud(*cloud, best_indexs, *cloud_ransac);
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handle(cloud_ransac, 0.0, 200.0, 0.0);
        // viewer->addPointCloud(cloud_ransac, color_handle, "cloud_ransac" + cloud->header.frame_id);
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_ransac" + cloud->header.frame_id);

        return Coeff;
    }

    pcl::PointXYZ get_waypoint()
    {
    }

    void loop()
    {
    }

private:
    void distance_callback(const std_msgs::msg::Float32::SharedPtr dis)
    {
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
    }

    // node
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_distance;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

// main
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AMRcontroler>());
    if (!rclcpp::ok())
    {
        system("ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' -1");
    }
    rclcpp::shutdown();
    return 0;
}
