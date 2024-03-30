#include <memory>
#include <string>
#include <iomanip>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
using std::placeholders::_1;
#define BOOST_BIND_NO_PLACEHOLDERS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>

class EJ_RegionGrowing: public pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>{
  public:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      getColoredCloud (){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
      
        if (!clusters_.empty ())
        {
          colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
      
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
          for (auto i_segment = clusters_.cbegin (); i_segment != clusters_.cend (); i_segment++)
          {
            for (auto i_point = i_segment->indices.cbegin (); i_point != i_segment->indices.cend (); i_point++)
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
              if(*(input_->points[index].data+2)>0.05&&*(input_->points[index].data+2)<1.5)
              {
                colored_cloud->points.push_back (point);
                colored_cloud->width++;
              }
            }
            // next_color++;
          }
        }
      
        return (colored_cloud);
            }
};


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("segmentation pcl"));
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("realtime pcl"));
class MinimalSubscriber : public rclcpp::Node
{
public:
//my struct
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
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "horizon", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    //my struct
    /*std::vector<pointCloud> pcs;
    auto pc=reinterpret_cast<pointCloud*> (msg->data.data());
    std::cout<<"sizeof(pointCloud) "<<sizeof(pointCloud)<<std::endl;
    std::cout<<pc->x<<" ,"<<pc->y<<" ,"<<pc->z<<std::endl;
    for (size_t i = 0; i < 24000; i++)
    {
      pcs.push_back(*pc);
      pc++;
    }*/
    
    //pcl 
    pcl::PCLPointCloud2 *pointCloud2 =new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*msg,*pointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pointCloud2,*cloud);
    // pcl::visualization::CloudViewer viewer("3D viewer");
    // viewer.showCloud(cloud);
    /*check data size*/
    // RCLCPP_INFO(this->get_logger(), "test: '%i'", msg->data.size());
    
    //segmentation
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);
  
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);
  
    EJ_RegionGrowing reg;
    // pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);    
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    // std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    // std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
    // std::cout << "These are the indices of the points of the initial" <<
    // std::endl << "cloud that belong to the first cluster:" << std::endl;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    //
    
    viewer1->removeAllPointClouds();
    // viewer1->addPointCloud(cloud,"realtime pcl");
    // viewer1->updatePointCloud(cloud,"realtime pcl");
    viewer1->addPointCloud(colored_cloud,"segmentation pcl");
    viewer1->updatePointCloud(colored_cloud,"segmentation pcl");
    viewer1->spinOnce(0.001);

    // viewer2->removeAllPointClouds();
    // viewer2->addPointCloud(cloud,"realtime pcl");
    // viewer2->updatePointCloud(cloud,"realtime pcl");
    // viewer2->spinOnce(0.001);

    //my-segmentation
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
