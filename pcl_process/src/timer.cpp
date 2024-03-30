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

        // 參數a或b不符合預期的濾除 //等待改成以切線斜率判斷 x=5, 2*ax+b
        //if (abs(2*5*a+b)<0.2)
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

class CloudSubscriber : public rclcpp::Node
{
public:
    CloudSubscriber()
        : Node("cloud_subscriber")
    {
        // 上幀曲線參數帶入差異容許範圍
        this->declare_parameter("threshold", 0.1);

        //
        this->declare_parameter("wall_distance", 3.0);

        this->declare_parameter("car_distance", 4.0);

        this->declare_parameter("x_target", 7.0);

        right << FLT_MAX, FLT_MAX, FLT_MAX;
        left << FLT_MAX, FLT_MAX, FLT_MAX;

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "horizon", 1, std::bind(&CloudSubscriber::topic_callback, this, _1));
        subscription_distance = this->create_subscription<std_msgs::msg::Float32>(
            "distance", 1, std::bind(&CloudSubscriber::distance_callback, this, _1));

        timer_ = this->create_wall_timer(
            500ms, std::bind(&CloudSubscriber::timer_callback, this));

        // geometry_msgs/msg/Twist
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void timer_callback()
    {
        auto drive_msg = geometry_msgs::msg::Twist();

        //
        // if(distance_2wall)
        //

        // 拋物線頂點[-b/2a,（4ac-b²）/4a]
        end = clock();
        // error = this->get_parameter("wall_distance").as_double();
        // dt = double(end - prev_time) / CLOCKS_PER_SEC;
        // prev_time = end;
        // integral += prev_error * dt;
        // derivative = (error - prev_error) / dt;
        // steering_angle = kp * error + ki * integral + kd * derivative;
        // prev_error = error;
        auto x_target = this->get_parameter("x_target").as_double();
        auto y_target = right(0) * x_target * x_target + right(1) * x_target + right(2) + this->get_parameter("wall_distance").as_double();
        auto temp_sin = x_target - this->get_parameter("car_distance").as_double();
        auto delta_up = 2 * this->get_parameter("car_distance").as_double() * (y_target / (temp_sin * temp_sin));
        auto delta_down = sqrt(temp_sin * temp_sin + y_target * y_target);
        drive_msg.angular.z = atan(delta_up / delta_down);
        drive_msg.linear.x = 2.0;

        publisher_->publish(drive_msg);
    }

    void distance_callback(const std_msgs::msg::Float32::SharedPtr dis) {
        distance_2wall=dis->data;
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ros raw pointcloud2格式轉換pcl raw pointcloudxyz
        pcl::PCLPointCloud2 *pointCloud2 = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*msg, *pointCloud2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ros(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*pointCloud2, *cloud_ros);

        // 準備分解pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_right->header.frame_id = "right";
        cloud_left->header.frame_id = "left";

        // 去高度化 z歸0
        for (size_t i = 0; i < cloud_ros->points.size(); i++)
        {
            // 只抓取高度為光達的+-1公尺內點雲，且xy非0，離x距離25公尺內
            if (cloud_ros->points[i].z > -1 && cloud_ros->points[i].z < 1 && cloud_ros->points[i].x && cloud_ros->points[i].x <= 25 && cloud_ros->points[i].y)
            {
                pcl::PointXYZ point;
                point.x = cloud_ros->points[i].x;
                point.y = cloud_ros->points[i].y;
                point.z = 0;
                cloud->points.push_back(point);

                // 障礙物判斷
                if (point.x < 3 && point.y <= 1 && point.y >= -1)
                {
                    cloud_obstacle->points.push_back(point);
                }

                // y>0為左邊，反之為右邊
                if (point.y > 0)
                {
                    // float vaule = point.x * point.x * left(0) + point.x * left(1) + left(2) - point.y;
                    // if (left(0) == FLT_MAX)
                    // {
                    cloud_left->points.push_back(point);
                    // }
                    // else if (abs(vaule) <= this->get_parameter("threshold").as_double())
                    // {
                    //     cloud_left->points.push_back(point);
                    // }
                }
                else
                {
                    // float vaule = point.x * point.x * right(0) + point.x * right(1) + right(2) - point.y;
                    // if (right(0) == FLT_MAX)
                    // {
                    cloud_right->points.push_back(point);
                    // }
                    // else if (abs(vaule) <= this->get_parameter("threshold").as_double())
                    // {
                    //     cloud_right->points.push_back(point);
                    // }
                }
            }
        }

        // PCL viewer 清理
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // viewer->addCoordinateSystem(1.0);
        viewer->addPointCloud(cloud, "cloud");

        if (cloud_right->size() > 3)
            right = ransac(cloud_right, 0.1, right);
        if (cloud_left->size() > 3)
            left = ransac(cloud_left, 0.1, left);

        // pcl::PointCloud<pcl::PointXYZ>::Ptr middle(new pcl::PointCloud<pcl::PointXYZ>);
        // for (size_t x_middle = 5; x_middle <= 15; x_middle++)
        // {
        //     pcl::PointXYZ point;
        //     point.x = x_middle;
        //     point.y = right(0) * x_middle * x_middle + right(1) * x_middle + right(2) + this->get_parameter("wall_distance").as_double();
        //     point.z = 0;
        //     middle->points.push_back(point);
        // }

        // viewer->addPointCloud(middle, "middle");

        viewer->spinOnce(0.001);
    }

    // 變數們
    Vector3f right, left; // 紀錄左右牆曲線參數
    double distance_2wall;
    string robot_stauts;

    // PID參數
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

    // node
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
        system("ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' -1");
    }
    rclcpp::shutdown();
    return 0;
}