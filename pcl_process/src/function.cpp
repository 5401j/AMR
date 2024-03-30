pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){

    std::vector<int> boundary_bool(cloud->size(), 0);
    double r = 1.8;
    pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
    kdtree.setInputCloud(cloud_in);
    std::vector<int> indices(0,0);
    std::vector<float>dist(0, 0.0);

    for (size_t i = 0; i < cloud_in->size(); ++i)
    {
        pcl::PointXYZ p = cloud_in->points[i];
        kdtree.radiusSearch(cloud_in->points[i], 2*r, indices,dist ,100);
        //indices[0]对应的点云即为查询点本身
        for (size_t j = 1; j < indices.size(); ++j)
        {
            pcl::PointXYZ p1 = cloud_in->points[indices[j]];
            double s_2 = std::pow((p.x - p1.x), 2) +
                std::pow((p.y - p1.y), 2);
            double h = std::pow((r * r / s_2 - 0.25), 0.5);
            double x2 = p.x + 0.5 * (p1.x - p.x) - h * (p1.y - p.y);
            double y2 = p.y + 0.5 * (p1.y - p.y) - h * (p.x - p1.x);
            double x3 = p.x + 0.5 * (p1.x - p.x) + h * (p1.y - p.y);
            double y3 = p.y + 0.5 * (p1.y - p.y) + h * (p.x - p1.x);
            pcl::PointXYZ p2(x2, y2, 0.0);
            pcl::PointXYZ p3(x3, y3, 0.0);
            //计算邻域内除了p1之外的点到p2,p3的距离
            std::vector<double>distp2,distp3;
            std::vector<int>distp2_bool(0, 0), distp3_bool(0, 0);
            int count = 0;
            for (size_t k = 1; k < indices.size(); ++k)
            {
                pcl::PointXYZ p_other = cloud_in->points[indices[k]];
                if (k != j)
                {
                    ++count;
                    double distance_p2 = get2DDist(p_other, p2);
                    double distance_p3 = get2DDist(p_other, p3);
                    //比较距离与r的大小
                    if (distance_p2 > r)
                    {
                        distp2_bool.push_back(1);
                    }
                    if (distance_p3 > r)
                    {
                        distp3_bool.push_back(1);
                    }
                }
            }
            //如果邻域内所有点到p2，或者p3的距离均大于r，则有distp2_bool.size()==count
            //则表明p是边界点,退出循环，不用再计算邻域内点距离了
            if (count == distp2_bool.size() || count == distp3_bool.size())
            {
                boundary_bool[i] = 1;
                break;
            }
        }      
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud;
    for (size_t it=0;it< boundary_bool->size();++it)
    {
        if (boundary_bool[it] == 1)
        {
            boundary_cloud->points.push_back(cloud_in->points[it]);
        }
    }
    boundary_cloud->height = boundary_cloud->size();
    boundary_cloud->width = 1;

    return boundary_cloud;
}