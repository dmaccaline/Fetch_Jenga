//include basic pcl items
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//include basic ros and conversions from pcl to ros
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
//include filters for cluster recognition
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//include transform broadcasters
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl/visualization/cloud_viewer.h>
#include <math.h>


tf::StampedTransform transform2;

void callback(const sensor_msgs::PointCloud2ConstPtr& input){
    //used for testing function
    //ROS_INFO("Data Recieved");

    tf::TransformBroadcaster br;
    tf::Transform transform;

    //create new pcl clouds to store data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //convert from sensor_msgs to pcl
    pcl::fromROSMsg(*input, *cloud_filtered);

    //sdetup for euclidean cluster ifentification
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.01); // 2cm
    ec.setMinClusterSize (150);
    ec.setMaxClusterSize (350);
    //ec.setMinClusterSize (2800);
    //ec.setMaxClusterSize (35000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);


    //int j, incremented through each loop, used to increase number at end of tf frame name, so each has unique name
    int j = 0;
    std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        //loop unil no new cluster found
        //creat pcl object to store found cloud temporarily
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud <pcl::PointXYZ>);
        for (const auto &idx : it->indices)
            cloud_cluster->push_back((*cloud_filtered)[idx]);
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        //print number of points in cloud
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        //create string to name cloud, using j to have incrementing number
        std::stringstream ss;
        ss << "cluster_" << j;

        //create variables to hold positions, used to averagae all points to find center of object
        int num_points = cloud_cluster->size();
        double x = 0;
        double y = 0;
        double z = 0;
        //create variables to fiind extreme points, used for rotation
        //double maxx = cloud_cluster->points[0].x;
        double maxy = cloud_cluster->points[0].y;
        double maxyX = cloud_cluster->points[0].x;
        double minx = cloud_cluster->points[0].x;
        double minxY = cloud_cluster->points[0].y;
        double miny = cloud_cluster->points[0].y;
        double minyX = cloud_cluster->points[0].x;
        //increment through points, adding xyz vcalues and finding extremes
        for (int p = 0; p < cloud_cluster->points.size(); ++p) {
            x = x + cloud_cluster->points[p].x;
            y = y + cloud_cluster->points[p].y;
            z = z + cloud_cluster->points[p].z;
            //if(cloud_cluster->points[p].x > maxx){
            //    maxx = cloud_cluster->points[p].x;
            //}
            if(cloud_cluster->points[p].x < minx){
                minx = cloud_cluster->points[p].x;
                minxY = cloud_cluster->points[p].y;
            }
            if(cloud_cluster->points[p].y > maxy){
                maxy = cloud_cluster->points[p].y;
                maxyX = cloud_cluster->points[p].x;
            }
            else if(cloud_cluster->points[p].y < miny){
                miny = cloud_cluster->points[p].y;
                minyX = cloud_cluster->points[p].x;
            }
        }

        //find average for x/y/z
        y = y / num_points;
        z = z / num_points;
        x = x / num_points;

        double result = 0;
        double slope;
        // find distance from miny to minx and minx to maxy (side lengths)
        double lengthOne = sqrt((minyX - minx) * (minyX - minx) + (miny-minxY) * (miny-minxY));
        double lengthTwo = sqrt((maxyX - minx) * (maxyX - minx) + (maxy-minxY) * (maxy-minxY));
       //determine which side is shorter, and use that side for calculating the angle
        if(lengthTwo > lengthOne){
            //slope = (miny-minxY)/(minyX - minx);
            slope = (minyX - minx)/(miny-minxY);
           // ROS_INFO("%.2lf %.2lf\n%.2lf %.2lf", minyX, minx, miny, minxY);
           // ROS_INFO("Slope1 = %.2lf", slope);
        }
        else{
            //slope = (maxy-minxY)/(maxyX - minx);
            slope = (maxyX - minx)/(maxy-minxY);
            //ROS_INFO("%.2lf %.2lf\n%.2lf %.2lf", maxyX, minx, maxy, minxY);
           // ROS_INFO("Slope2 = %.2lf", slope);
        }
        //set result as atan of the found slope, returns the angle of the slope in radians
        result = atan(slope);

        //setup and run transform
        tf::Quaternion q;
        //cmath isnan

        //use ,matrix to get yaw, used to correct for head turned away from center
        tf::Matrix3x3 m(transform2.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        //ROS_WARN("roll = %.2lf\npitch = %.2lf\nyaw = %.2lf", roll, pitch, yaw);

        //test if q is a valid number
        if(!isnan(result)){
            q.setEulerZYX(yaw + result + 1.57,0,0);
            //q.setEulerZYX(-2.36 + result + 1.57,0,0);
        }
        else{
            ROS_WARN("nan");
            break;
        }
        //ROS_WARN("Result: %.2lf", result);


        tf::Vector3 point(x, y, z);

        tf::Vector3 base_point = transform2 * point;
        transform.setRotation(q);
        transform.setOrigin(base_point);
        //transform.setOrigin(point);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", ss.str()));

        //Break statement here as only one tf frame is needed, comment out to see all frames found
        break;

        //increment j
        j++;

    }

    //print number of found clouds
    ROS_INFO("Number of clouds: %d", j);

}
int main(int argc, char** argv) {
    ros::init(argc, argv, "cropped_planar_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(4);

    //get transform before callback
    tf::TransformListener listener;

    try {
        listener.waitForTransform("/base_link", "/head_camera_depth_optical_frame", ros::Time(0),
                                  ros::Duration(10.0));
        listener.lookupTransform("/base_link", "/head_camera_depth_optical_frame", ros::Time(0), transform2);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("Failed to find transform betwenn /head_camera_depth_optical_frame and /base_link: %s",
                  ex.what());
        //exit file if error in transform found
        return 0;
    }

    ROS_INFO("Running");
    tf::TransformBroadcaster tester;

    ros::Subscriber sub = nh.subscribe("planar_seg_down", 1000, callback);

    //used for testing test_movement (in moveit_demo)
    //ros::Subscriber sub = nh.subscribe("cropped_point", 1000, callback);
    ros::spin();
    return 0;
}
