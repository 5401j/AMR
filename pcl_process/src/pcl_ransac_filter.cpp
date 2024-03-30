#include <iostream>
#include <memory>
#include <string>
#include <iomanip>
#include <vector>
#include <chrono>
#include <time.h>
#include <cmath>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
#define BOOST_BIND_NO_PLACEHOLDERS
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace Eigen;

#define PI 3.1415926

struct color
{
  unsigned char R;
  unsigned char G;
  unsigned char B;
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ransac"));

bool ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::VectorXf &coef)
{
  if (cloud->size() < 10)
  {
    return false;
  }
  // std::cout << " ransac." << std::endl;
  pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr line(new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(cloud));
  line->setAxis(Vector3f(1, 0, 0));
  line->setEpsAngle(10 * PI / 180);
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(line);
  ransac.setDistanceThreshold(0.1);
  ransac.setMaxIterations(log(1 - 0.99) / log(1 - pow(0.2, 3)));
  bool model = ransac.computeModel();
  // std::cout << " ransac.computeModel();" << std::endl;
  vector<int> inliers;
  ransac.getInliers(inliers);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_line);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colour_handle(cloud_line, 254, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud_line, colour_handle, "ransac_" + cloud->header.frame_id);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ransac_" + cloud->header.frame_id);

  Vector4f pt;
  pcl::getMaxDistance(*cloud_line, Vector4f(0, 0, 0, 1), pt);
  pcl::PointXYZ p2(pt(0), pt(1), 0);
  pcl::getMaxDistance(*cloud_line, pt, pt);
  pcl::PointXYZ p1(pt(0), pt(1), 0);

  // cout << cloud->header.frame_id << "直線上兩點座標: " << p1 << ", " << p2 << endl;
  viewer->addLine(p1, p2, 100, 0, 100, "line_" + cloud->header.frame_id, 0);
  if (model)
    ransac.getModelCoefficients(coef);

  return model;
}

void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, PointCloud<PointXYZ>::Ptr &cloud_right, PointCloud<PointXYZ>::Ptr &cloud_left)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.1);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(2500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  // int j = 0;
  for (const auto &cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &idx : cluster.indices)
    {
      cloud_cluster->push_back((*cloud)[idx]);
    }
    pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr line(new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(cloud_cluster));
    line->setAxis(Vector3f(1, 0, 0));
    line->setEpsAngle(20 * PI / 180);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(line);
    ransac.setDistanceThreshold(0.1);
    ransac.setMaxIterations(1000);
    if (ransac.computeModel())
    {
      vector<int> inliers;
      ransac.getInliers(inliers);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::copyPointCloud<pcl::PointXYZ>(*cloud_cluster, inliers, *cloud_line);
      Eigen::VectorXf coef;
      ransac.getModelCoefficients(coef);
      double m = coef[4] / coef[3];
      double b = -m * coef[0] + coef[1];
      if (b > 0)
      {
        *cloud_left += *cloud_cluster;
      }
      else
      {
        *cloud_right += *cloud_cluster;
      }
    }

    // j++;
  }
}

class CloudSubscriber : public rclcpp::Node
{
public:
  CloudSubscriber()
      : Node("Cloud_subscriber")
  {
    coef_right.resize(6);
    coef_right << FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX;
    coef_left.resize(6);
    coef_left << FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX;

    this->declare_parameter("threshold", 0.1);

    subscription_horizon = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "horizon", 1, std::bind(&CloudSubscriber::topic_callback, this, _1));

    subscription_distance = this->create_subscription<std_msgs::msg::Float32>(
        "demo/distance_demo", 1, std::bind(&CloudSubscriber::distance_callback, this, _1));

    subscription_status = this->create_subscription<std_msgs::msg::String>(
        "mission_status", 1, std::bind(&CloudSubscriber::status_callback, this, _1));

    // timer_ = this->create_wall_timer(
    //     500ms, std::bind(&CloudSubscriber::timer_callback, this));

    // geometry_msgs/msg/Twist
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("demo/cmd_demo", 10);
  }

private:
  double kp = 0.01;
  double kd = 0.0;
  double ki = 0.0;
  clock_t start = clock();
  clock_t end = clock();
  clock_t prev_time = clock();
  double prev_error = 0.0;
  double error = 0.0;
  double integral = 0.0;
  double dt = 0.0;
  double derivative = 0.0;
  double steering_angle = 0.0;

  Eigen::VectorXf coef_right;
  Eigen::VectorXf coef_left;
  double m_right, b_right, m_left, b_left;
  bool right_, left_;

  double distanceCB;
  double temp_disCB = -30;
  char status = 'F'; // stop go
  double sum_right = 0.0;
  double sum_left = 0.0;
  size_t count_head = 0;
  double dis_right = 0.0;
  double dis_left = 0.0;
  char rol;

  void timer_callback()
  {
    auto drive_msg = geometry_msgs::msg::Twist();
    cout << "status: " << status << endl;
    cout << "里程計數據：" << distanceCB << "-" << temp_disCB << endl;
    switch (status)
    {
    case 'S':
      drive_msg.linear.x = 0.0;
      steering_angle = 0.0;
      break;

    case 'G':
      drive_msg.linear.x = 1.5;
      end = clock();
      dt = double(end - prev_time) / CLOCKS_PER_SEC;
      prev_time = end;
      integral += prev_error * dt;
      double m_angle;
      if (rol == 'R')
      {
        m_angle = m_right;
        error = -dis_right + b_right;
      }
      else
      {
        m_angle = m_left;
        error = b_left - dis_left;
      }
      derivative = (error - prev_error) / dt;
      steering_angle = kp * error + ki * integral + kd * derivative;
      prev_error = error;

      std::cout << "steering_angle=" << steering_angle << std::endl;
      if (steering_angle > 0.2)
      {
        steering_angle = 0.2;
      }
      else if (steering_angle < -0.2)
      {
        steering_angle = -0.2;
      }

      if (abs(m_angle) > tan(20 * PI / 180.0f))
      {
        steering_angle = 0.8 * m_angle + 0.2 * steering_angle;
      }

      break;

    case 'F':
      drive_msg.linear.x = 1.5;
      steering_angle += 0.0;
      break;
    }

    drive_msg.angular.z = steering_angle;
    publisher_->publish(drive_msg);
  }

  void distance_callback(const std_msgs::msg::Float32::SharedPtr dis)
  {
    distanceCB = dis->data;
    if (distanceCB - temp_disCB >= 30)
    {

      status = 'S';
      temp_disCB = distanceCB;
    }
    else if (distanceCB - temp_disCB >= 10 && dis_right == 0 && dis_left == 0)
    {
      status = 'S';
      temp_disCB = distanceCB;

      dis_right = sum_right / count_head;
      dis_left = sum_left / count_head;
      cout << dis_left << "," << dis_right << endl;
    }
  }
  void status_callback(const std_msgs::msg::String::SharedPtr str)
  {
    if (str->data == "done")
    {
      if (distanceCB < 10)
        status = 'F';
      else
        status = 'G';
    }
    else
    {
      status = 'S';
    }
  }

  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

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
      if (cloud_ros->points[i].z > -1 && cloud_ros->points[i].z < 1 && cloud_ros->points[i].x && cloud_ros->points[i].y) //&& cloud_ros->points[i].x <= 30
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
          float vaule = point.x * m_left + b_left - point.y;
          if (coef_left[0] == FLT_MAX)
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
          float vaule = point.x * m_right + b_right - point.y;
          if (coef_right[0] == FLT_MAX)
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
    // cluster(cloud, cloud_right, cloud_left);

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addCoordinateSystem(1.0);

    rol = cloud_left->size() > cloud_right->size() ? 'L' : 'R';

    // cout << "點雲尺寸:" << cloud->size() << ",左：" << cloud_left->size() << ",右：" << cloud_right->size() << endl;

    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handle1(cloud_right, 0.0, 200.0, 0.0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_right, color_handle1, "cloud_right");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handle2(cloud_left, 0.0, 0.0, 200.0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_left, color_handle2, "cloud_left");

    // right_ = ransac(cloud_right, coef_right);
    // left_ = ransac(cloud_left, coef_left);

    // m_right = coef_right[4] / coef_right[3];
    // b_right = -m_right * coef_right[0] + coef_right[1];

    // m_left = coef_left[4] / coef_left[3];
    // b_left = -m_left * coef_left[0] + coef_left[1];

    // if (distanceCB <= 10)
    // {
    //   sum_right += b_right;
    //   sum_left += b_left;
    //   count_head++;
    // }
    // else
    // {
    //   dis_right = sum_right / count_head;
    //   dis_left = sum_left / count_head;
    // }

    viewer->spinOnce(0.001);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_horizon;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_distance;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_status;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

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
