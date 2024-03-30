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
using std::placeholders::_1;
#define BOOST_BIND_NO_PLACEHOLDERS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace Eigen;

#define PI 3.1415926

struct color {
    unsigned char R;
    unsigned char G;
    unsigned char B;
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ransac"));

class EJ_RegionGrowing: public pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>{
  public:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      getColoredCloud (){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

           srand (static_cast<unsigned int> (time (nullptr)));
          std::vector<unsigned char> colors;
          for (std::size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
          {
            colors.push_back (static_cast<unsigned char> (rand () % 256));
            colors.push_back (static_cast<unsigned char> (rand () % 256));
            colors.push_back (static_cast<unsigned char> (rand () % 256));
          }
      
        if (!clusters_.empty ())
        {
          colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
      
      
          colored_cloud->width = 0;
          colored_cloud->height = 1;
          colored_cloud->is_dense = input_->is_dense;
          int next_color = 0;

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
              point.r = colors[3 * next_color];
              point.g = colors[3 * next_color + 1];
              point.b = colors[3 * next_color + 2];
              // point.r = 255;
              // point.g = 0;
              // point.b = 0;
              if(*(input_->points[index].data+2)>0.05&&*(input_->points[index].data+2)<1.5)
              {
                colored_cloud->points.push_back (point);
                colored_cloud->width++;
              }
            }
            next_color++;
          }
        }
      
        return (colored_cloud);
  }
  
};

std::vector<pcl::PointIndices> cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {
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

void first_ransac(PointCloud<pcl::PointXYZ>::Ptr& cloud,int index, PointCloud<PointXYZ>::Ptr &cloud_right, PointCloud<PointXYZ>::Ptr& cloud_left) {
    // std::cout << " first_ransac." << std::endl;
    pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr line(new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(cloud));
    //line->setAxis(Vector3f(1, 0, 0));
    //line->setEpsAngle(0.15);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(line);
    ransac.setDistanceThreshold(0.01);
    ransac.setMaxIterations(1000);
    ransac.computeModel();

    vector<int> inliers;
    ransac.getInliers(inliers);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_line);

    Eigen::VectorXf coef;
    ransac.getModelCoefficients(coef);

    double m = coef[4] / coef[3];
    double b = -m * coef[0] + coef[1];
    if (cloud_line->size()>2 && m<tan(10* PI / 180.0f) && m > tan(-10 * PI / 180.0f)) {

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


Eigen::VectorXf second_ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::cout << " second_ransac." << std::endl;
    pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr line(new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(line);
    line->setAxis(Vector3f(1, 0, 0));
    line->setEpsAngle(10*PI/180);
    ransac.setDistanceThreshold(0.1);
    ransac.setMaxIterations(1000);
    ransac.computeModel();

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


    cout << cloud->header.frame_id <<"直線上兩點座標: " << p1 << ", " << p2 << endl;
    viewer->addLine(p1, p2, 100, 0, 100, "line_" + cloud->header.frame_id, 0);

    Eigen::VectorXf coef;
    ransac.getModelCoefficients(coef);

    return coef;
}

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("test_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "horizon", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    
    //pcl 
    pcl::PCLPointCloud2 *pointCloud2 =new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*msg,*pointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pointCloud2,*cloud);
    
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

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    // viewer->addPointCloud(colored_cloud,"RegionGrowing pcl");
    // viewer->updatePointCloud(colored_cloud,"RegionGrowing pcl");

    //ransac
    PointCloud<PointXYZ>::Ptr cloud_right(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloud_left(new PointCloud<PointXYZ>);
    cloud_right->header.frame_id = "right";
    cloud_left->header.frame_id = "left";

    // auto cluster_indices=cluster(cloud_boundary);
    int j = 0;
    for (const auto& cluster : clusters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster.indices) {
          if((*cloud)[idx].z>0.05&&(*cloud)[idx].z<1.5){
            (*cloud)[idx].z=0;
            cloud_cluster->push_back((*cloud)[idx]);
          }
        }
        if(cloud_cluster->size()>0){
        color c{ rand() % 256, rand() % 256, rand() % 256 };
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handle(cloud_cluster, c.R, c.G, c.B);
        viewer->addPointCloud(cloud_cluster, color_handle, "cloud_" + to_string(j));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_" + to_string(j) );

        first_ransac(cloud_cluster, j, cloud_right, cloud_left);
        j++;
        }
    }

    // second_ransac
    Eigen::VectorXf coef_right=second_ransac(cloud_right);
    Eigen::VectorXf coef_left=second_ransac(cloud_left);



    viewer->spinOnce(0.001);

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
