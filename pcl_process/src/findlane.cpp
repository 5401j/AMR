#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>     
#include <cmath>
#include <time.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;
using namespace pcl;
pcl::visualization::PCLVisualizer viewer("Viewer");
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
        vector<int> cur_ptsIdx(allPts);			//提取當前直線後，不在當前直線上的其他點索引
        for (int i = 0; i < allPts; i++)
            cur_ptsIdx[i] = i;

        int r = 0;
        vector<int> line_ptsIdx, all_ptsIdx(allPts);
        vector<vector<float>> lines;			//所有直線參數 [0]: k, [1]: b
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
            //cout << "第 " << r++ << " 次循環,直線參數: " << best_lineModel << endl;
            //   cout<<"所有點的個數:  "<<cur_ptsIdx.size()<<endl;
            //   cout<<"當前直線上的點數："<<line_ptsIdx.size()<<endl;

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
        /*pcl::visualization::PCLVisualizer viewer("Viewer");*/
        viewer.setBackgroundColor(0.5, 0.5, 0.5, 0);
        viewer.addPointCloud(cloud, to_string(cloud->size()));
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, to_string(cloud->size()));
        for (int i = 0; i < lines.size(); i++) {
            pcl::PointXYZ p1(0, 0 * lines[i][0] + lines[i][1], 0);
            pcl::PointXYZ p2(30, 30 * lines[i][0] + lines[i][1], 0);
            cout << "直線上兩點座標: " << p1 << ", " << p2 << endl;
            viewer.addLine(p1, p2, 240 - 40 * i, 0, 0, "line:m=" + to_string((p1.y - p2.y) / (p1.x - p2.x)), 0);
            //cout << "斜率:  " << (p1.y - p2.y) /(p1.x-p2.x) << endl;
        }
        //while (!viewer.wasStopped()) {
        //    viewer.spinOnce();
        //}
    }

};
void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_1 , pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_2) {
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

int main() {
    clock_t start=clock(), end;
    
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	io::loadPCDFile("D:\\Downloads\\20230905.pcd", *cloud);
	PointCloud<PointXYZ>::Ptr cloud_1(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_2(new PointCloud<PointXYZ>);
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end();) {
		// std::cout<<"it: "<< it->x<<std::endl;
		/*if (it->x* it->x+ it->y* it->y > 25*25)cloud->erase(it);*/
        if (it->x  > 20)cloud->erase(it);
		else it++;
	}
    /*
	for (int i_point = 0; i_point < cloud->size(); i_point++)
	{
        if (cloud->points[i_point].x <= 20) {
            cloud_all->points.push_back(cloud->points[i_point]);
			if (cloud->points[i_point].y > 0) {
				cloud_left->points.push_back(cloud->points[i_point]);
			}
			else{
				cloud_right->points.push_back(cloud->points[i_point]);
			}
        }
	}
    */
    cluster(cloud, cloud_1, cloud_2);
    fitLineRansac ransac;
    //ransac.ransac_line(cloud, 7, 50);
    ransac.ransac_line(cloud_1, 7, 50);
    ransac.ransac_line(cloud_2, 7, 50);
    end = clock();
    std::cout << double(end - start) / CLOCKS_PER_SEC << "秒" << std::endl;
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    return 0;
}