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

std::vector<pcl::PointIndices> cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(1);
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(2500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  return cluster_indices;
}

void first_ransac(PointCloud<pcl::PointXYZ>::Ptr &cloud, int index, PointCloud<PointXYZ>::Ptr &cloud_right, PointCloud<PointXYZ>::Ptr &cloud_left)
{
  // std::cout << " first_ransac." << std::endl;
  pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr line(new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(cloud));
  // line->setAxis(Vector3f(1, 0, 0));
  // line->setEpsAngle(0.15);
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(line);
  ransac.setDistanceThreshold(0.1);
  ransac.setMaxIterations(1000);
  ransac.computeModel();
  // std::cout << " first_ransac.computeModel();" << std::endl;
  vector<int> inliers;
  ransac.getInliers(inliers);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_line);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colour_handle(cloud_line, 254, 0, 0);

  Eigen::VectorXf coef;
  ransac.getModelCoefficients(coef);

  double m = coef[4] / coef[3];
  double b = -m * coef[0] + coef[1];
  if (cloud_line->size() > 2 && m < tan(30 * PI / 180.0f) && m > tan(-30 * PI / 180.0f))
  {

    if (b > 0)
    {
      *cloud_left += *cloud_line;
    }
    else
    {
      *cloud_right += *cloud_line;
    }
  }
}

bool second_ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::VectorXf &coef)
{
  // std::cout << " second_ransac." << std::endl;
  pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr line(new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(line);
  line->setAxis(Vector3f(1, 0, 0));
  line->setEpsAngle(20 * PI / 180);
  ransac.setDistanceThreshold(0.05);
  ransac.setMaxIterations(1000);
  bool model = ransac.computeModel();
  // std::cout << " second_ransac.computeModel();" << std::endl;
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

class FollowerSubscriber : public rclcpp::Node
{
public:
  FollowerSubscriber()
      : Node("test_subscriber")
  {
    this->declare_parameter("KS", 50.0); // setKSearch
    this->declare_parameter("RS", 0.1);  // setRadiusSearch
    this->declare_parameter("AT", 1.2);  // setAngleThreshold
    this->declare_parameter("DIS", 7.0);
    this->declare_parameter("CH", false);

    subscription_horizon = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "horizon", 1, std::bind(&FollowerSubscriber::topic_callback, this, _1));

    subscription_distance = this->create_subscription<std_msgs::msg::Float32>(
        "demo/distance_demo", 1, std::bind(&FollowerSubscriber::distance_callback, this, _1));

    // geometry_msgs/msg/Twist
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("demo/cmd_demo", 10);
  }

private:
  double my_setKSearch;
  double my_setRadiusSearch;
  double my_setAngleThreshold;
  double my_distance;
  double my_choice;

  char status = 'G'; // G 行動 //M 測量
  size_t station = 0;
  bool finish = false;

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

  double distanceCB = 0.0;
  double temp_disCB = 0.0;
  double pre_disCB = 0.0;
  double sum_right = 0.0;
  double sum_left = 0.0;
  size_t count_head = 0;
  double dis_right = 0.0;
  double dis_left = 0.0;

  void distance_callback(const std_msgs::msg::Float32::SharedPtr dis)
  {
    distanceCB = dis->data;
    // cout << "distanceCB: " << distanceCB << endl;
    size_t dd = int(distanceCB);
    // auto drive_msg = geometry_msgs::msg::Twist();

    if (temp_disCB > 10 || station == 0)
    {
      temp_disCB = 0;
      status = 'M';
      station++;
      
      // drive_msg.linear.x = 0;
      // drive_msg.angular.z = 0;
      // publisher_->publish(drive_msg);
      system("ros2 topic pub /demo/cmd_demo geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' -1");
      rclcpp::WallRate loop_rate(10000ms);
      loop_rate.sleep();
    }else status = 'G';

    // if (status == 'M'&& !finish)
    // {
    //   cout << "第"<< station <<"站開始任務!! " <<endl;
    //   cout << "第"<< station <<"站結束任務!! " <<endl;
    //   cin >>finish;
    //   status = 'G'; station++;

    // }
    // else if( status == 'G' && distanceCB>1 && int(distanceCB)%10==9 && finish ){
    //   finish = false;
    // }else if( status == 'G' && int(distanceCB)%10==0 && !finish && station!=0){
    //   status = 'M';

    // }
    temp_disCB += (distanceCB - pre_disCB);
    pre_disCB = distanceCB;
  }

  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // cout << "機器人測站模式: " << status << endl;
    auto drive_msg = geometry_msgs::msg::Twist();

    my_setKSearch = this->get_parameter("KS").as_double();
    my_setRadiusSearch = this->get_parameter("RS").as_double();
    my_setAngleThreshold = this->get_parameter("AT").as_double();
    my_distance = this->get_parameter("DIS").as_double();
    my_choice = this->get_parameter("CH").as_bool();

    start = clock();
    prev_time = start;

    pcl::PCLPointCloud2 *pointCloud2 = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*msg, *pointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ros(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(*pointCloud2, *cloud_ros);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    PointCloud<PointXYZ>::Ptr cloud_obstacle(new PointCloud<PointXYZ>);
    // 去高度化 z歸0
    for (size_t i = 0; i < cloud_ros->points.size(); i++)
    {
      if (cloud_ros->points[i].z > -1 && cloud_ros->points[i].z < 1 && cloud_ros->points[i].x && cloud_ros->points[i].y)
      {
        pcl::PointXYZ point;
        point.x = *(cloud_ros->points[i].data);
        point.y = *(cloud_ros->points[i].data + 1);
        point.z = 0;
        cloud->points.push_back(point);
        if (point.x < 3 && point.y <= 1 && point.y >= -1)
        {
          cloud_obstacle->points.push_back(point);
        }
      }
    }

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    // viewer->addPointCloud(cloud, "cloud");
    
    viewer->addPointCloud(cloud_obstacle, "cloud_obstacle");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud_obstacle");
    // cout << "cloud_obstacle size():" << cloud_obstacle->size() << endl;

    if (distanceCB > 10 && status == 'G')
    {
      if (!dis_left && !dis_right)
      {
        dis_left = sum_left / count_head;
        dis_right = sum_right / count_head;
        // cout << "左邊距離" << dis_left << ",右邊距離" << dis_right << endl;
      }

      // boundary estimation
      //  estimate normals and fill in \a normalsmsg
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
      normal_estimator.setInputCloud(cloud);
      normal_estimator.setKSearch(my_setKSearch);
      normal_estimator.compute(*normals);

      pcl::PointCloud<pcl::Boundary> boundaries;
      pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
      est.setInputCloud(cloud);
      est.setInputNormals(normals);
      est.setRadiusSearch(my_setRadiusSearch); // 10cm radius
      est.setAngleThreshold(M_PI * my_setAngleThreshold);
      est.setSearchMethod(pcl::search::KdTree<PointXYZ>::Ptr(new pcl::search::KdTree<PointXYZ>));
      est.compute(boundaries);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
      for (size_t i = 0; i < cloud->points.size(); i++)
      {

        if (boundaries[i].boundary_point > 0)
        {
          cloud_boundary->push_back(cloud->points[i]);
          color c{rand() % 256, rand() % 256, rand() % 256};
        }
      }

      viewer->addPointCloud(cloud_boundary, "cloud_boundary");
      viewer->updatePointCloud(cloud_boundary, "cloud_boundary");

      // viewer->addPointCloud(cloud, "cloud");
      // viewer->updatePointCloud(cloud, "cloud");

      // ransac
      PointCloud<PointXYZ>::Ptr cloud_right(new PointCloud<PointXYZ>);
      PointCloud<PointXYZ>::Ptr cloud_left(new PointCloud<PointXYZ>);
      cloud_right->header.frame_id = "right";
      cloud_left->header.frame_id = "left";
      auto cluster_indices = cluster(cloud_boundary);
      int j = 0;
      for (const auto &cluster : cluster_indices)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices)
        {
          cloud_cluster->push_back((*cloud_boundary)[idx]);
        }

        color c{rand() % 256, rand() % 256, rand() % 256};
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handle(cloud_cluster, c.R, c.G, c.B);
        // viewer->addPointCloud(cloud_cluster, color_handle, "cloud_" + to_string(j));
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_" + to_string(j));

        first_ransac(cloud_cluster, j, cloud_right, cloud_left);
        j++;
      }

      color c{rand() % 256, rand() % 256, rand() % 256};
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handle1(cloud_right, c.R, c.G, c.B);
      viewer->addPointCloud(cloud_right, color_handle1, "cloud_right");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_right");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handle2(cloud_left, c.R, c.G, c.B);
      viewer->addPointCloud(cloud_left, color_handle2, "cloud_left");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_left");
      // second_ransac
      bool right = second_ransac(cloud_right, coef_right);
      bool left = second_ransac(cloud_left, coef_left);

      double pre_b_right = b_right;
      double pre_b_left = b_left;

      // cout << "coef mR: " << coef_right << endl;

      m_right = coef_right[4] / coef_right[3];
      b_right = -m_right * coef_right[0] + coef_right[1];

      // cout << "coef mL: " << coef_left << endl;
      m_left = coef_left[4] / coef_left[3];
      b_left = -m_left * coef_left[0] + coef_left[1];

      if (isnan(m_right))
        m_right = m_left;
      if (isnan(m_left))
        m_left = m_right;

      end = clock();
      // std::cout << double(end - start) / CLOCKS_PER_SEC << "秒" << std::endl;
      char PIDcase = 'S';
      double steering_angle;
      // case S:停止不動 ,R: 右邊pid＋左右斜率平均 ,L: 左邊pid＋左右斜率平均 ,A: 左右斜率平均
      // 1:右牆存在且沒遇避車灣，左牆存在且沒遇避車灣（右邊pid）.
      // 2:右牆存在但遇到避車灣，左牆存在且沒遇避車灣 (左邊pid).
      // 3:右牆存在但遇到避車灣，左牆存在但遇到避車灣 (停止不動).
      // 4:右牆不存在，左牆存在且沒遇避車灣(左邊pid)
      // 5:右牆不存在，左牆存在但遇到避車灣(停止不動)
      // 6:兩道牆皆不存在（停止不動）.
      // 7:左牆不存在，右牆存在且沒遇避車灣（右邊pid）
      // 8:左牆不存在，右牆存在但遇到避車灣（停止不動）
      // 9:有障礙物（停止不動）.
      // cout << cloud_obstacle->size() << "  ,  " << right << "  ,  " << left;
      if (!(cloud_obstacle->size() > 100))
      {
        if (right && left)
        {
          // cout << "  ,  " << b_right - pre_b_right << "  ,  " << dis_right - b_right;
          // cout << "  ,  " << b_left - pre_b_left << "  ,  " << dis_left - b_left << endl;
          if (abs(b_right - pre_b_right) < 2 && abs(dis_right - b_right) < 2)
            PIDcase = 'R';
          else if (abs(b_left - pre_b_left) < 2 && abs(dis_left - b_left) < 2)
            PIDcase = 'L';
        }
        else if (right)
        {
          if (abs(b_right - pre_b_right) < 2 && abs(dis_right - b_right) < 2)
            PIDcase = 'R';
        }
        else if (left)
        {
          if (abs(b_left - pre_b_left) < 2 && abs(dis_left - b_left) < 2)
            PIDcase = 'L';
        }
      }

      end = clock();
      cout << "case:" << PIDcase << "     /";
      switch (PIDcase)
      {
      case 'R':
        error = -dis_right + b_right;
        dt = double(end - prev_time) / CLOCKS_PER_SEC;
        prev_time = end;
        integral += prev_error * dt;
        derivative = (error - prev_error) / dt;
        steering_angle = kp * error + ki * integral + kd * derivative;

        if (m_right > tan(20 * PI / 180.0f))
          steering_angle = m_right;
        else if (m_left > tan(10 * PI / 180.0f))
          steering_angle = 0.2 * steering_angle + 0.8 * m_right;
        else
          steering_angle = 0.5 * steering_angle + 0.5 * m_right;

        prev_error = error;

        drive_msg.linear.x = 1.0;
        break;

      case 'L':
        error = b_left - dis_left;
        dt = double(end - prev_time) / CLOCKS_PER_SEC;
        prev_time = end;
        integral += prev_error * dt;
        derivative = (error - prev_error) / dt;
        steering_angle = kp * error + ki * integral + kd * derivative;

        if (m_left > tan(20 * PI / 180.0f))
          steering_angle = m_left;
        else if (m_left > tan(10 * PI / 180.0f))
          steering_angle = 0.2 * steering_angle + 0.8 * m_left;
        else
          steering_angle = 0.5 * steering_angle + 0.5 * m_left;

        prev_error = error;

        drive_msg.linear.x = 1.0;
        break;

      default:
        drive_msg.linear.x = 0.0;
        steering_angle = 0.0;
        break;
      }

      if (steering_angle > 0.2)
      {
        steering_angle = 0.2;
      }
      else if (steering_angle < -0.2)
      {
        steering_angle = -0.2;
      }

      // drive_msg.angular.z = 0.4*steering_angle + 0.5*((m_right + m_left) / 2);
      drive_msg.angular.z = steering_angle;
      cout << "steering_angle: kp*" << error << "  aka, " << dis_right << "-" << b_right << "/  ";
      cout << " 斜率： " << m_left << "/" << m_right << " ,截距： " << b_left << "/" << b_right << " ,轉向： " << steering_angle << endl;
    }
    else if (status == 'G')
    {
      PointCloud<PointXYZ>::Ptr cloud_right(new PointCloud<PointXYZ>);
      PointCloud<PointXYZ>::Ptr cloud_left(new PointCloud<PointXYZ>);
      cloud_right->header.frame_id = "right";
      cloud_left->header.frame_id = "left";
      for (int i_point = 0; i_point < cloud->size(); i_point++)
      {
        if (cloud->points[i_point].x <= 20)
        {
          if (cloud->points[i_point].y > 0)
          {
            cloud_left->points.push_back(cloud->points[i_point]);
          }
          else
          {
            cloud_right->points.push_back(cloud->points[i_point]);
          }
        }
      }

      second_ransac(cloud_right, coef_right);
      second_ransac(cloud_left, coef_left);

      m_right = coef_right[4] / coef_right[3];
      b_right = -m_right * coef_right[0] + coef_right[1];

      m_left = coef_left[4] / coef_left[3];
      b_left = -m_left * coef_left[0] + coef_left[1];

      sum_right += b_right;
      sum_left += b_left;
      count_head++;

      if (cloud_obstacle->size() > 100)
      {
        drive_msg.linear.x = 0;
        drive_msg.angular.z = 0;
      }
      else
      {
        drive_msg.linear.x = 1;
        drive_msg.angular.z = 0;
      }
    }

    publisher_->publish(drive_msg);
    viewer->spinOnce(0.001);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_horizon;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_distance;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerSubscriber>());
  if (!rclcpp::ok())
  {
    system("ros2 topic pub /demo/cmd_demo geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' -1");
  }
  rclcpp::shutdown();
  return 0;
}
