#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <iostream>

int main(int argc, char** argv)
{
    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/grvc/Escritorio/scans_bueno.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the PCD file");
        return -1;
    }
    std::cout << "Filtering outliers with Statistical Outlier Removal..." << std::endl;
    // Create the Statistical Outlier Removal filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(400);  // Set number of nearest neighbors to use
    sor.setStddevMulThresh(0.5);  // Set standard deviation multiplier threshold
    // Points with a distance larger than mean + stddev * threshold are considered outliers and removed
    sor.filter(*cloud_filtered);

    // std::cout << "Filtering outliers with Radius Outlier Removal..." << std::endl;
    // // Create the Radius Outlier Removal filter
    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    // ror.setInputCloud(cloud);
    // ror.setRadiusSearch(0.2);  // Set radius for neighbor search
    // ror.setMinNeighborsInRadius(200);  // Minimum number of neighbors within the radius
    // ror.filter(*cloud_filtered2);

    std::cout << "Saving filtered PDC files..." << std::endl;
    pcl::io::savePCDFileASCII("output_filtered_sor.pcd", *cloud_filtered);
    // pcl::io::savePCDFileASCII("output_filtered_ror.pcd", *cloud_filtered2);
    std::cout << "Done." << std::endl;
    return 0;
}
