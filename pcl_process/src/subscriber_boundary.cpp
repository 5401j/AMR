#include <memory>
#include <string>
#include <iomanip>
#include <vector>
#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
using std::placeholders::_1;
#define BOOST_BIND_NO_PLACEHOLDERS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>

using namespace pcl;
using namespace pcl::io;

class EJ_RegionGrowing : public pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>
{
public:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  getColoredCloud()
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

    if (!clusters_.empty())
    {
      colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

      // srand (static_cast<unsigned int> (time (nullptr)));
      // std::vector<unsigned char> colors;
      // for (std::size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
      // {
      //   colors.push_back (static_cast<unsigned char> (rand () % 256));
      //   colors.push_back (static_cast<unsigned char> (rand () % 256));
      //   colors.push_back (static_cast<unsigned char> (rand () % 256));
      // }

      colored_cloud->width = 0;
      colored_cloud->height = 1;
      colored_cloud->is_dense = input_->is_dense;
      // for (std::size_t i_point = 0; i_point < input_->points.size (); i_point++)
      // {
      //   pcl::PointXYZRGB point;
      //   point.x = *(input_->points[i_point].data);
      //   point.y = *(input_->points[i_point].data + 1);
      //   point.z = *(input_->points[i_point].data + 2);
      //   point.r = 255;
      //   point.g = 0;
      //   point.b = 0;
      //   colored_cloud->points.push_back (point);
      // }
      // int next_color = 0;
      for (auto i_segment = clusters_.cbegin(); i_segment != clusters_.cend(); i_segment++)
      {
        for (auto i_point = i_segment->indices.cbegin(); i_point != i_segment->indices.cend(); i_point++)
        {
          int index;
          index = *i_point;
          pcl::PointXYZRGB point;
          point.x = *(input_->points[index].data);
          point.y = *(input_->points[index].data + 1);
          point.z = 0;
          // point.r = colors[3 * next_color];
          // point.g = colors[3 * next_color + 1];
          // point.b = colors[3 * next_color + 2];
          point.r = 255;
          point.g = 0;
          point.b = 0;
          if (*(input_->points[index].data + 2) > 0.05 && *(input_->points[index].data + 2) < 1.5)
          {
            colored_cloud->points.push_back(point);
            colored_cloud->width++;
          }
        }
        // next_color++;
      }
    }

    return (colored_cloud);
  }
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("pcl"));
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("realtime pcl"));
class MinimalSubscriber : public rclcpp::Node
{
public:
  // my struct
  /*
  #pragma pack(1)
    struct pointCloud{
      float x;
      float y;
      float z;
      float intensity;
      uint8_t tag;
      uint8_t line;
      };
  */

  MinimalSubscriber()
      : Node("test_subscriber")
  {
    this->declare_parameter("KS", 50.0); // setKSearch
    this->declare_parameter("RS", 0.1);  // setRadiusSearch
    this->declare_parameter("AT", 1.2);  // setAngleThreshold

    my_setKSearch = this->get_parameter("KS").as_double();
    my_setRadiusSearch = this->get_parameter("RS").as_double();
    my_setAngleThreshold = this->get_parameter("AT").as_double();

    this->set_parameters(std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("KS", 50.0),
        rclcpp::Parameter("RS", 0.1),
        rclcpp::Parameter("AT", 1.2)});

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "horizon", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  double my_setKSearch;
  double my_setRadiusSearch;
  double my_setAngleThreshold;
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    // std::cout << "my_setRadiusSearch:" << my_setRadiusSearch << std::endl;
    // pcl

    pcl::PCLPointCloud2 *pointCloud2 = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*msg, *pointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ros(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(*pointCloud2, *cloud_ros);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < cloud_ros->points.size(); i++)
    {
      if (*(cloud_ros->points[i].data + 2) > -1 && *(cloud_ros->points[i].data + 2) < 1 && cloud_ros->points[i].x && cloud_ros->points[i].y)
      {
        pcl::PointXYZ point;
        point.x = *(cloud_ros->points[i].data);
        point.y = *(cloud_ros->points[i].data + 1);
        point.z = 0;
        cloud->points.push_back(point);
      }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    pcl::io::savePCDFileASCII("20231115.pcd", *cloud);
    // pcl::visualization::CloudViewer viewer("3D viewer");
    // viewer.showCloud(cloud);
    // check data size
    // RCLCPP_INFO(this->get_logger(), "test: '%i'", msg->data.size());

    // boundary estimation

    // estimate normals and fill in \a normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(my_setKSearch);
    normal_estimator.compute(*normals);

    // RegionGrowing

    // pcl::IndicesPtr indices (new std::vector <int>);
    // pcl::removeNaNFromPointCloud(*cloud, *indices);
    // EJ_RegionGrowing reg;
    // reg.setMinClusterSize (50);
    // reg.setMaxClusterSize (1000000);
    // reg.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
    // reg.setNumberOfNeighbours (30);
    // reg.setInputCloud (cloud);
    // reg.setIndices (indices);
    // reg.setInputNormals (normals);
    // reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    // reg.setCurvatureThreshold (1.0);
    // std::vector <pcl::PointIndices> clusters;
    // reg.extract (clusters);
    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud_o = reg.getColoredCloud ();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // copyPointCloud(*colored_cloud_o, *colored_cloud);

    // normal_estimator.setInputCloud (colored_cloud);
    // normal_estimator.setKSearch (50);
    // normal_estimator.compute (*normals);

    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud(cloud);
    est.setInputNormals(normals);
    est.setRadiusSearch(my_setRadiusSearch); // 10cm radius
    est.setAngleThreshold(M_PI * my_setAngleThreshold);
    est.setSearchMethod(pcl::search::KdTree<PointXYZ>::Ptr(new pcl::search::KdTree<PointXYZ>));
    est.compute(boundaries);

    // std::cout<<boundaries<<"\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    // for (int i = 0; i < colored_cloud->points.size(); i++)
    // {

    // 	if (boundaries[i].boundary_point > 0)
    // 	{
    // 		cloud_boundary->push_back(colored_cloud->points[i]);
    // 	}
    // }

    for (int i = 0; i < cloud->points.size(); i++)
    {

      if (boundaries[i].boundary_point > 0)
      {
        cloud_boundary->push_back(cloud->points[i]);
      }
    }

    // output pcd
    //  time_t rawtime;
    //  struct tm * timeinfo;
    //  char buffer[80];

    // time (&rawtime);
    // timeinfo = localtime(&rawtime);
    // strftime(buffer,sizeof(buffer),"%d%m%H%M",timeinfo);
    // std::string strr(buffer);
    // std::cout<<"strr:"<<strr<<std::endl;

    // pcl::io::savePCDFileASCII ("20231029.pcd", *cloud);

    // view

    // int v1(0);
    // view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    // view->setBackgroundColor(0.3, 0.3, 0.3, v1);
    // view->addText("Raw point clouds", 10, 10, "v1_text", v1);
    // int v2(0);
    // view->createViewPort(0.5, 0.0, 1, 1.0, v2);
    // view->setBackgroundColor(0.5, 0.5, 0.5, v2);
    // view->addText("Boudary point clouds", 10, 10, "v2_text", v2);
    // view->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
    // view->addPointCloud<pcl::PointXYZ>(cloud_boundary, "cloud_boundary", v2);
    // view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
    // view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);
    // view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_boundary",v2);
    view->addText("param:" + std::to_string(my_setRadiusSearch), 10, 10, "text");
    view->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    view->addPointCloud<pcl::PointXYZ>(cloud_boundary, "cloud_boundary");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_boundary");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_boundary");
    view->addCoordinateSystem(1.0);
    view->initCameraParameters();

    view->spin();
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
