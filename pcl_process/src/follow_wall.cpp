#include <memory>
#include <string>
#include <iomanip>
#include <vector>
#include <chrono>
#include <time.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;
#define BOOST_BIND_NO_PLACEHOLDERS
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
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

using namespace std;
using namespace pcl;
using namespace pcl::io;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ransac"));

class fitLineRansac
{
  public:

    vector<vector<float>> ransac_line(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float dist, int iterate) 
    {
        /***
         *		dist: 點到直線距離小於dist 即當前點在直線上
         *      iterate: 迭代次數
         ***/
        int allPts = cloud->points.size();
        vector<vector<float>> lines;			//所有直線參數 [0]: k, [1]: b
        if(allPts==0) return lines;
        vector<int> cur_ptsIdx(allPts);			//提取當前直線後，不在當前直線上的其他點索引
        for (int i = 0; i < allPts; i++)
            cur_ptsIdx[i] = i;

        int r = 0;
        vector<int> line_ptsIdx, all_ptsIdx(allPts);
        vector<float> cur_line(2);
        Eigen::Vector3f line_model, best_lineModel;
        while (1)
        {
            int line_pts = 0, tmp_pts;
            if (r >= 2) iterate = iterate / 3;
            if (cur_ptsIdx.size() < 10 && cur_ptsIdx.size() > 3) iterate = 4;
            for (int i = 0; i < iterate; i++) {
                line_model = leastSquare(cloud, cur_ptsIdx, dist);
                tmp_pts = line_model[2] / 1;
                if (tmp_pts > line_pts) {
                    line_pts = tmp_pts;
                    best_lineModel = line_model;
                    line_ptsIdx = tmp_ptsIdx;
                }
                tmp_ptsIdx.clear();
            }

            cur_line[0] = best_lineModel[0]; cur_line[1] = best_lineModel[1];
            lines.push_back(cur_line);
            r++;

            //得到剩餘點的索引
            for (int i = 0; i < line_ptsIdx.size(); i++)
                all_ptsIdx[line_ptsIdx[i]] = 1;
            cur_ptsIdx.clear();
            for (int j = 0; j < allPts; j++)
                if (!all_ptsIdx[j]) cur_ptsIdx.push_back(j);

            if (cur_ptsIdx.size() < 5) {
                break;
            }
        }
        view(cloud, lines);
        return lines;
    }
    private:
    
    vector<int> tmp_ptsIdx;			//當前直線上的點數
    Eigen::Vector3f leastSquare(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int> pIdx, float dist)
    {
        //求解給定若干點的直線方程
        float a = 0, B = 0, c = 0, d = 0;          //a: x之和　b: y之和  c: x平方和  d: x*y之和  e: 樣本數量
        int s = pIdx.size();
        vector<int> cur_ptsIdx = rangedRand(0, s, 4);					//4：每次選４點用最小二乘擬合直線
        int e = cur_ptsIdx.size();
        for (int i = 0; i < e; i++) {
            a += cloud->points[pIdx[cur_ptsIdx[i]]].x;
            B += cloud->points[pIdx[cur_ptsIdx[i]]].y;
            c += cloud->points[pIdx[cur_ptsIdx[i]]].x * cloud->points[pIdx[cur_ptsIdx[i]]].x;
            d += cloud->points[pIdx[cur_ptsIdx[i]]].x * cloud->points[pIdx[cur_ptsIdx[i]]].y;
        }
        float k, b;
        float tmp = e * c - a * a;
        if (abs(tmp) > 0.0005) {
            b = (c * B - a * d) / tmp;
            k = (e * d - a * B) / tmp;
        }
        else {
            k = 1; b = 0;
        }

        //求每一個點到直線的距離，小於dist, 即在直線上
        int line_pnum = 0;
        for (int i = 0; i < s; i++) {
            float d, numerator, denominator;             //分子分母        點到直線的距離　d = |kx - y + b| / sqrt(k^2 + 1)
            numerator = abs(k * cloud->points[pIdx[i]].x - cloud->points[pIdx[i]].y + b);
            denominator = sqrt(k * k + 1);
            d = numerator / denominator;
            if (d < dist) {
                line_pnum++;
                tmp_ptsIdx.push_back(pIdx[i]);
            }
        }
        Eigen::Vector3f line_model;
        line_model[0] = k; line_model[1] = b; line_model[2] = line_pnum;
        return line_model;
    }

    vector<int> rangedRand(int range_begin, int range_size, int n)
    {
        int i; vector<int> indices;
        // srand((unsigned)time(NULL));           //生成隨機種子
        for (i = 0; i < n; i++)
        {
            int u = rand() % range_size + range_begin; //生成[range_begin, range_begin+range_siz]內的隨機數
            // cout<<i<<": "<<u<<endl;
            indices.push_back(u);
        }
        return indices;
    }

    void view(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<vector<float>> lines)
    {
    //   for (int i = 0; i < lines.size(); i++) {
        if(lines.size()==1){
            pcl::PointXYZ p1(0, 0 * lines[0][0] + lines[0][1], 0);
            pcl::PointXYZ p2(30, 30 * lines[0][0] + lines[0][1], 0);
            cout << "直線上兩點座標: " << p1 << ", " << p2 << endl;
            viewer->addLine(p1, p2, 240, 0, 0, "line:m=" + to_string(lines[0][0]), 0);
            cout << "斜率:  " << (p1.y - p2.y) /(p1.x-p2.x) << endl;
        }
    }

};
void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_1 , pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_2) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(2); 
	ec.setMinClusterSize(10);
	ec.setMaxClusterSize(2500);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);
    
    int j = 0;
    for (const auto& cluster : cluster_indices)
	{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	    for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*cloud)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;
        
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
	    std::stringstream ss;
	    ss << std::setw(4) << std::setfill('0') << j;
        switch (j)
        {
        case 0:
            cloud_1 = cloud_cluster;
            std::cout << "Cluster1: " << cloud_1->size() << " data points." << std::endl;
            break;
        case 1:
            cloud_2 = cloud_cluster;
            std::cout << "Cluster2: " << cloud_2->size() << " data points." << std::endl;
            break;
        default:
            break;
        }
        //writer.write<pcl::PointXYZ>("cloud_cluster_" + ss.str() + ".pcd", *cloud_cluster, false); //*
        j++;
    }
}


class MinimalSubscriber : public rclcpp::Node
{
public:

  MinimalSubscriber()
  : Node("test_subscriber")
  {
    this->declare_parameter("KS", 50.0 );//setKSearch
    this->declare_parameter("RS", 0.1 );//setRadiusSearch
    this->declare_parameter("AT", 1.2);//setAngleThreshold

    my_setKSearch = this->get_parameter("KS").as_double();
    my_setRadiusSearch= this->get_parameter("RS").as_double();
    my_setAngleThreshold = this->get_parameter("AT").as_double();

    this->set_parameters(std::vector<rclcpp::Parameter>{
    rclcpp::Parameter("KS", 50.0),
    rclcpp::Parameter("RS", 0.1),
    rclcpp::Parameter("AT", 1.2)
    });

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "horizon", 1, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    // geometry_msgs/msg/Twist
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("demo/cmd_demo", 10);
  }

private:
    double my_setKSearch;
    double my_setRadiusSearch;
    double my_setAngleThreshold;

    double kp = 0.6;
    double kd = 0.0;
    double ki = 0.0;
    clock_t start =clock();
    clock_t end=clock();
    clock_t prev_time=clock();
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
  
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    start=clock();prev_time=start;
    
    pcl::PCLPointCloud2 *pointCloud2 =new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*msg,*pointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ros(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(*pointCloud2,*cloud_ros);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //去高度化 z歸0
    for (size_t i = 0; i < cloud_ros->points.size(); i++)
    {
      if(*(cloud_ros->points[i].data+2)>0.05&&*(cloud_ros->points[i].data+2)<1.5){
        pcl::PointXYZ point;
        point.x = *(cloud_ros->points[i].data);
        point.y = *(cloud_ros->points[i].data + 1);
        point.z = 0;
        cloud->points.push_back(point);
      }
    }
    
    //boundary estimation
    // estimate normals and fill in \a normals
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (my_setKSearch);
    normal_estimator.compute (*normals);

    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud (cloud);
    est.setInputNormals (normals);
    est.setRadiusSearch (my_setRadiusSearch);   // 10cm radius
    est.setAngleThreshold(M_PI * my_setAngleThreshold);
    est.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
    est.compute (boundaries);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->points.size(); i++)
  	{

  		if (boundaries[i].boundary_point > 0)
  		{
  			cloud_boundary->push_back(cloud->points[i]);
  		}
  	}
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud(cloud_boundary, "cloud_boundary");
    viewer->updatePointCloud(cloud_boundary, "cloud_boundary");
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_boundary->begin(); it != cloud_boundary->end();) {
		  if (it->x* it->x+ it->y* it->y > 25*25)cloud_boundary->erase(it);
		  else it++;
    }
    viewer->addPointCloud(cloud_boundary, "cloud_ransac");
    viewer->updatePointCloud(cloud_boundary, "cloud_ransac");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_ransac");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_ransac");
    PointCloud<PointXYZ>::Ptr cloud_1(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_2(new PointCloud<PointXYZ>);
    cluster(cloud_boundary, cloud_1, cloud_2);
    fitLineRansac ransac;
    auto line_1= ransac.ransac_line(cloud_1, 7, 50);
    auto line_2= ransac.ransac_line(cloud_2, 7, 50);
    // cout << "line_1斜率:  " << line_1[0][0] << endl;
    cout << "line_1截距:  " << line_1[0][1] << endl;
    cout << "line_2截距:  " << line_2[0][1] << endl; //負的是右牆 正的是左牆

    //wall follow
    end = clock();
    std::cout << double(end - start) / CLOCKS_PER_SEC << "秒" << std::endl;

    double dist=4.0;
    if(std::isnan(line_1[0][1])){
        line_1[0][1]=0;
    }
    if(std::isnan(line_2[0][1])){
        line_2[0][1]=0;
    }
    error = dist-abs(line_2[0][1]);
    double dt=double(end - prev_time) / CLOCKS_PER_SEC;
    prev_time=end;
    integral += prev_error * dt;
    double derivative = (error - prev_error) / dt;
    double steering_angle = kp * error + ki * integral + kd * derivative;
    prev_error = error;

    auto drive_msg=geometry_msgs::msg::Twist();

    if (std::isnan(steering_angle))
    {
        drive_msg.linear.x = 0;
        drive_msg.angular.z = 0;
        std::__throw_runtime_error("The Control Value to Steering cannot be nan");
    }
    std::cout<<"angular="<<steering_angle<<std::endl;
    if (steering_angle > 0.2){
        steering_angle = 0.2;
    }else if (steering_angle < -0.2){
        steering_angle = -0.2;
    }
    drive_msg.linear.x=1;
    drive_msg.angular.z=steering_angle;//正的左轉
    publisher_->publish(drive_msg);
    viewer->spinOnce(0.001);
    }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
