#include <iostream>
#include <vector>
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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace Eigen;

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
    std::cout << " first_ransac." << std::endl;
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
