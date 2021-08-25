#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

ros::Publisher pub;
//publishes downsampled version of fetch_camera view
//publishes to topic /fetch_downsampled

void callback(const sensor_msgs::PointCloud2ConstPtr& input){
    //ROS_INFO("Data Recieved");
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    //convert from sensor_msgs pointcloud to pcl pointcloud, store result in cloud
    pcl_conversions::toPCL (*input, *cloud);
    //print"numb. points before filter"
    //std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
    //          << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);
    //print numb. points after filter
    //std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
    //         << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
    //publish cloud_filtered to fetch_downsampled topic

    pub.publish(cloud_filtered);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "fetch_downsampler");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::PointCloud2> ("fetch_downsampled", 100);

    ros::Rate loop_rate(2);
    ros::Subscriber sub = nh.subscribe("head_camera/depth_registered/points", 1000, callback);

    ros::spin();
    return (0);
}