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

        // 參數a或b不符合預期的濾除
        // if (abs(a) > 0.01 || abs(b) > 0.15)
        // {
        //     t++;
        //     continue;
        // }

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

    pcl::copyPointCloud(*cloud, best_indexs, *cloud_ransac);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handle(cloud_ransac, 0.0, 200.0, 0.0);
    viewer->addPointCloud(cloud_ransac, color_handle, "cloud_ransac" + cloud->header.frame_id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_ransac" + cloud->header.frame_id);

    return Coeff;
}