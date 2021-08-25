#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;


void callback(const sensor_msgs::PointCloud2ConstPtr& input){
    //ROS_INFO("Data Recieved");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud_filtered);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);


    //remove back wall before segmentation
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-.2, 5.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);


    // Create the segmentation object
    pcl::SACSegmentation <pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold(0.01);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_filtered->size ();

    // While 30% of the original cloud is still there
    //.3
    while (cloud_filtered->size () > 0.2 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height
        //          << " data points." << std::endl;

        //std::stringstream ss;
        //ss << "table_scene_lms400_plane_" << i << ".pcd";
        //writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
        i++;
    }

    pub.publish(cloud_filtered);

}
int
main (int argc, char** argv) {
    ros::init(argc, argv, "planar_segmentation");
    ros::NodeHandle nh;

    ros::Rate loop_rate(1);
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("planar_seg_down", 100);


    ros::Subscriber sub = nh.subscribe("fetch_downsampled", 1000, callback);
    //used for testing
    //ros::Subscriber sub = nh.subscribe("head_camera/depth_registered/points", 1000, callback);
    //ros::Subscriber sub = nh.subscribe("head_camera/depth_downsample/points", 1000, callback);
    ros::spin();
    return (0);
}