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

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ransac"));

Vector3f ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double th, Vector3f Coeff)
{

    srand(time(NULL));
    cout << "ransac start\n";

    size_t iteration, t = 0;
    std::vector<int> best_indexs;
    pcl::PointXYZ min, max;

    getMinMax3D(*cloud, min, max);
    double param = pow(0.2, 3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ransac(new pcl::PointCloud<pcl::PointXYZ>);
    iteration = log(1 - 0.99) / log(1 - param);
    vector<int> bestInlierIndices;
    cout << iteration << endl;

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

        if (abs(a) > 0.01 || abs(b) > 0.15)
        {
            t++;
            continue;
        }
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

    cout << "實際內點比例" << best_indexs.size() << ":" << cloud->size() << "\n";

    pcl::copyPointCloud(*cloud, best_indexs, *cloud_ransac);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handle(cloud_ransac, 0.0, 200.0, 0.0);
    viewer->addPointCloud(cloud_ransac, color_handle, "cloud_ransac" + cloud->header.frame_id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_ransac" + cloud->header.frame_id);

    return Coeff;
}

class CloudSubscriber : public rclcpp::Node
{
public:
    CloudSubscriber()
        : Node("cloud_subscriber")
    {

        this->declare_parameter("threshold", 0.1);

        right << FLT_MAX, FLT_MAX, FLT_MAX;
        left << FLT_MAX, FLT_MAX, FLT_MAX;
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "horizon", 1, std::bind(&CloudSubscriber::topic_callback, this, _1));
        subscription_distance = this->create_subscription<std_msgs::msg::Float32>(
            "demo/distance_demo", 1, std::bind(&CloudSubscriber::distance_callback, this, _1));

        timer_ = this->create_wall_timer(
            500ms, std::bind(&CloudSubscriber::timer_callback, this));

        // geometry_msgs/msg/Twist
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("demo/cmd_demo", 10);
    }

private:
    Vector3f right, left;

    double kp = 0.1;
    double kd = 0.0;
    double ki = 0.0;
    clock_t prev_time = clock();
    clock_t end = clock();
    double error = 0.0;
    double prev_error = 0.0;
    double integral = 0.0;
    double dt = 0.0;
    double derivative = 0.0;
    double steering_angle = 0.0;

    double distanceCB;
    double temp = 0.0;
    double sum_right = 0.0;
    double sum_left = 0.0;
    size_t count_head = 0;
    double dis_right = 0.0;
    double dis_left = 0.0;
    bool set_dis = 0;

    // size_t station=0;

    void timer_callback()
    {
        if (!set_dis && distanceCB >= 10)
        {
            dis_left = sum_left / count_head;
            dis_right = sum_right / count_head;
            cout << "左邊距離" << dis_left << ",右邊距離" << dis_right << endl;
            set_dis = 1;
        }

        auto drive_msg = geometry_msgs::msg::Twist();

        if (distanceCB >= 10)
        {
            drive_msg.linear.x = 0;
            // 拋物線頂點[-b/2a,（4ac-b²）/4a]
            end = clock();
            // error = abs(left(2))-abs(dis_left) ;
            error = abs(dis_right) - abs((4 * right(0) * right(2) - right(1) * right(1)) / (4 * right(0)));
            dt = double(end - prev_time) / CLOCKS_PER_SEC;
            prev_time = end;
            integral += prev_error * dt;
            derivative = (error - prev_error) / dt;
            steering_angle = kp * error + ki * integral + kd * derivative;
            prev_error = error;
            cout << "steering_angle:" << steering_angle << endl;
            // drive_msg.angular.z = steering_angle;
        }
        else
        {
            drive_msg.linear.x = 1.5;
            drive_msg.angular.z = 0.5 * right(1) + 0.5 * left(1);
        }
        // publisher_->publish(drive_msg);
    }
    void distance_callback(const std_msgs::msg::Float32::SharedPtr dis)
    {
        distanceCB = dis->data;
    }
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->addCoordinateSystem(1.0);

        pcl::PCLPointCloud2 *pointCloud2 = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*msg, *pointCloud2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ros(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromPCLPointCloud2(*pointCloud2, *cloud_ros);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        PointCloud<PointXYZ>::Ptr cloud_obstacle(new PointCloud<PointXYZ>);
        PointCloud<PointXYZ>::Ptr cloud_right(new PointCloud<PointXYZ>);
        PointCloud<PointXYZ>::Ptr cloud_left(new PointCloud<PointXYZ>);
        cloud_right->header.frame_id = "right";
        cloud_left->header.frame_id = "left";

        // 去高度化 z歸0
        for (size_t i = 0; i < cloud_ros->points.size(); i++)
        {
            if (cloud_ros->points[i].z > -1 && cloud_ros->points[i].z < 1 && cloud_ros->points[i].x && cloud_ros->points[i].x <= 25 && cloud_ros->points[i].y)
            {
                pcl::PointXYZ point;
                point.x = cloud_ros->points[i].x;
                point.y = cloud_ros->points[i].y;
                point.z = 0;
                cloud->points.push_back(point);
                if (point.x < 3 && point.y <= 1 && point.y >= -1)
                {
                    cloud_obstacle->points.push_back(point);
                }

                if (point.y > 0)
                {
                    float vaule = point.x * point.x * left(0) + point.x * left(1) + left(2) - point.y;
                    if (left(0) == FLT_MAX)
                    {
                        cloud_left->points.push_back(point);
                    }
                    else if (abs(vaule) <= this->get_parameter("threshold").as_double())
                    {
                        cloud_left->points.push_back(point);
                    }
                }
                else
                {
                    float vaule = point.x * point.x * right(0) + point.x * right(1) + right(2) - point.y;
                    if (right(0) == FLT_MAX)
                    {
                        cloud_right->points.push_back(point);
                    }
                    else if (abs(vaule) <= this->get_parameter("threshold").as_double())
                    {
                        cloud_right->points.push_back(point);
                    }
                }
            }
        }
        viewer->addPointCloud(cloud, "cloud");
        // viewer->addPointCloud(cloud_right, "cloud_right_p");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_right_p");
        if (cloud_right->size())
            right = ransac(cloud_right, 0.1, right);
        cout << "right:\n"
             << right << endl;

        if (cloud_left->size())
            left = ransac(cloud_left, 0.1, left);
        cout << "left:\n"
             << left << endl;

        if (distanceCB <= 10)
        {
            sum_right += right(2);
            sum_left += left(2);
            count_head++;
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handle(cloud_right, 200.0, 0.0, 0.0);
        viewer->addPointCloud(cloud_right, color_handle, "cloud_right");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_right");

        viewer->spinOnce(0.001);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_distance;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

// main
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudSubscriber>());
    if (!rclcpp::ok())
    {
        system("ros2 topic pub /demo/cmd_demo geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' -1");
    }
    rclcpp::shutdown();
    return 0;
}