#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

int main()
{
    // Загрузка из файлa
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/dima846/ros2_ws/src/testTask/test_cloud.pcd", *cloud);
    
    
    // Фильтрация земли с использованием алгоритма RANSAC
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ > seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filtered_cloud);

    
      // Формирование изображения
    float resolution = 0.5;
    cv::Mat bev_image(200, 200, CV_32FC1, cv::Scalar(0.0));

    for (const auto& point : filtered_cloud->points)
    {
        float x = point.x;
        float y = point.y;

        if (x >= -100 && x <= 100 && y >= -100 && y <= 100)
        {
            int col = static_cast<int>((x + 100) / resolution);
            int row = static_cast<int>((y + 100) / resolution);
            bev_image.at<float>(row, col) += 1.0;
        }
    }
    
    // Нормализация интенсивности пикселей
    double min, max;
    cv::minMaxLoc(bev_image, &min, &max);
    bev_image = (bev_image - min) / (max - min);

    // Визуализация изображения
    cv::imshow("BEV Image", bev_image);
    cv::waitKey(0);
    
    return 0;
}
