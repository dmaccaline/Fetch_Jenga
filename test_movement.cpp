#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf/transform_datatypes.h>
#include <control_msgs/GripperCommandAction.h>
#include <std_srvs/Empty.h>

#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#include <control_msgs/PointHeadAction.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
//include basic pcl items
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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

#include <math.h>

//stores input data iniversally for use by other functions
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 cloud_filtered_two;

//used for testing
//ros::Publisher pub2;
//ros::Publisher pub3;

//stack block one on top of each other, uses run for height, no perception
void setDefaultDrop(int run, double height, tf::Quaternion* g, geometry_msgs::Pose* dropLocation);
//defailt drop varient, that can detect highest block in certain area and target that block for dropping
//also detects angle to work even if block is rotated
void defaultDropPerception(tf::Quaternion* g, geometry_msgs::Pose* dropLocation);

//2x2 tower, stack around given point and angle.  No perception, uses run number for height and orientation
void stackDropLocation(int run, double height, tf::Quaternion* g, geometry_msgs::Pose* dropLocation);
//varient of stack drop that adds an offset to adjust for gripper grabbing closer to one end of a block
void stackDropLocationOffset(int run, double height, tf::Quaternion* g, geometry_msgs::Pose* dropLocation);
//varient of stackDropLocation that uses perception to determine drop location
void stackDropPerception(tf::Quaternion* g, geometry_msgs::Pose* dropLocation, double height);

//set target location for grasping block on table, target is at block itself
void setTarget(tf::Quaternion q, geometry_msgs::Pose* target, tf::StampedTransform transform);
//apply offset to geometry_msg Pose of amount amt, in the Z (verticle) direction, used when moving up/down toward block
geometry_msgs::Pose offset(double amt, geometry_msgs::Pose position, tf::Quaternion g);


void callback(const sensor_msgs::PointCloud2ConstPtr& input){
    //take input and store as pcl cloud in cloud_filtered and sensor_msg in cloud_filtered_two, to make available for whole file
    pcl::fromROSMsg(*input, *cloud_filtered);
    cloud_filtered_two = *input;

}
int main(int argc, char** argv) {
    ros::init(argc, argv, "test_movement");
    ros::NodeHandle nh;
    //needed to get current robot state
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber sub;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("/rh/points_for_moveit_octomap", 100);
    //pub 2/3 used for testing
    //pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/cropped_point", 100);
    //pub3 = nh.advertise<sensor_msgs::PointCloud2> ("/additional_field", 100);
    //pub = nh.advertise<sensor_msgs::PointCloud2> ("/bin", 100);

    //setup planning scene
    moveit::planning_interface::MoveGroupInterface group("arm_with_torso");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    //record start position (to move arm back to start when needed
    std::vector<double> group_variable_orig_values;
    group.getCurrentState()->copyJointGroupPositions(
            group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_orig_values);

    //setup rviz tools for wait
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    //stores run number for non-perception drop methods
    int run = 0;

    //initialize services
    //head service
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> hm("head_controller/point_head", true);
    ROS_INFO("Waiting for action server to start.");
    hm.waitForServer();
    sleep(1);
    ROS_INFO("Action server started.");
    control_msgs::PointHeadGoal head_goal;

    //gripper service
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac("gripper_controller/gripper_action", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started.");
    control_msgs::GripperCommandGoal goal;

    goal.command.position = .5;
    goal.command.max_effort = 150;
    ac.sendGoal(goal);

    //planner
    moveit::planning_interface::MoveGroupInterface::Plan movement_plan;

    //initialize subscriber
    sub = nh.subscribe("fetch_downsampled", 1000, callback);
    //sub = nh.subscribe("full_cloud", 1000, callback);
    //loop until exited by ctr+c
    while(ros::ok()){

        //setup listener
        sleep(5);


        tf::TransformListener listener;
        tf::StampedTransform transform2;
        tf::StampedTransform transform1;

        ROS_INFO("trying transforms");
        //get transform between head_camera and trarget cluster for octomap cropping
        try
        {
            listener.waitForTransform("/head_camera_depth_optical_frame", "/cluster_0", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/head_camera_depth_optical_frame", "/cluster_0",  ros::Time(0), transform1);
            //listener.waitForTransform("/base_link", "/cluster_0", ros::Time(0), ros::Duration(10.0));
            //listener.lookupTransform("/base_link", "/cluster_0",  ros::Time(0), transform);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("ERROR in looking up transform for octomap", ex.what());
            //quit program if no lookup found
            break;
        }

        //get transform between head_camera and trarget cluster for grasping locations
        try
        {
            listener.waitForTransform("/base_link", "/cluster_0", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/base_link", "/cluster_0",  ros::Time(0), transform2);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("ERROR in looking up transform for arm movement", ex.what());
            //quit program if no lookup found
            break;
        }

        ROS_INFO("locations recieved, cropping octomap");

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        //crop field for octomap
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("y");
        //pass.setFilterLimits (-.05+-.11, .05+-.11);
        pass.setFilterLimits (transform1.getOrigin().y()-.1,transform1.getOrigin().y()+.1);
        pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_f);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("x");
        //pass.setFilterLimits (-.05 + -.19, .05 + -.19);
        pass.setFilterLimits (-.1 + transform1.getOrigin().x(), .1 + transform1.getOrigin().x());
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_p);

        *cloud = *cloud_p + *cloud_f;

        //wait for publisher to be ready, then publish octomap
        ROS_INFO("publishing octomap");
        while (pub.getNumSubscribers() < 1) {
        }
        int i = 0;
        while (i < 50){
            pub.publish(cloud);
            i++;
        }
        ROS_INFO("Published");
        //sleep(15);

        //setup quartenions fpr movement
        tf::Quaternion q;
        tf::Matrix3x3 m(transform2.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        //ROS_WARN("roll = %.2lf\npitch = %.2lf\nyaw = %.2lf", roll, pitch, yaw);
        q.setEulerZYX(yaw + 1.57, 1.57, roll);

        //create and set goal location
        geometry_msgs::Pose targetPose;
        setTarget(q, &targetPose, transform2);
        group.setPoseTarget(targetPose);
        //execute
        group.plan(movement_plan);
        visual_tools.prompt("Press 'next' to start moement");
        ROS_INFO("moving to goal");
        group.execute(movement_plan);

        //open gripper
        ROS_INFO("complete, opening gripper");

        goal.command.position = .1;
        goal.command.max_effort = 250;
        ac.sendGoal(goal);
        sleep(5);

        //move down to goal
        group.setPoseTarget(offset(-.22, targetPose, q));

        group.plan(movement_plan);
        visual_tools.prompt("Press 'next' to start moement");
        ROS_INFO("complete, moving down");
        group.execute(movement_plan);

        targetPose.position.x = targetPose.position.x;

        group.setPoseTarget(offset(-.22, targetPose, q));
        group.plan(movement_plan);
        visual_tools.prompt("Press 'next' to start moement");
        ROS_INFO("complete, normalizing");
        group.execute(movement_plan);

        //grasp attempt
        ROS_INFO("complete, gripping attempt");

        goal.command.position = 0;
        goal.command.max_effort = 150;
        ac.sendGoal(goal);
        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");

        //move up
        group.setPoseTarget(targetPose);

        group.plan(movement_plan);
        visual_tools.prompt("Press 'next' to start moement");
        ROS_INFO("complete, moving up");
        group.execute(movement_plan);

        //move arm out of way for perception if the arm is to the right side
        std::vector<double> getCurVals;
        group.getCurrentState()->copyJointGroupPositions(
                group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), getCurVals);

        ROS_INFO("Orig val %.2lf", getCurVals[1]);
        if(getCurVals[1] < 0){
            ROS_INFO("Arm is in the way of perception, moveing to the left");


            getCurVals[1] = 0;
            group.setJointValueTarget(getCurVals);

            group.plan(movement_plan);
            //                visual_tools.prompt("Press 'next' to begin arm movement");
            group.execute(movement_plan);
        }

        //add to right octomap
        visual_tools.prompt("Press 'next' to start head moement");
        ROS_INFO("Moving to right");

        head_goal.max_velocity = 1;
        head_goal.target.point.x = .7;
        head_goal.target.point.y = -.8;
        head_goal.target.point.z = .35555;
        head_goal.target.header.frame_id = "base_link";
        hm.sendGoal(head_goal);

        hm.sendGoal(head_goal);

        sleep(7);

        ROS_INFO("publishing to octomap");
        while (pub.getNumSubscribers() < 1) {
        }
        i = 0;
        while (i < 50){
            pub.publish(cloud_filtered);
            i++;
        }
        ROS_INFO("Published");

        //create variable for and set drop location

        //geometry_msgs::Pose dropLocation;
        sleep(2);
        tf::Quaternion g;
        geometry_msgs::Pose dropLocation;
/*
        try {
            listener.waitForTransform("/base_link", "/head_camera_depth_optical_frame", ros::Time(0),
                                      ros::Duration(10.0));
            listener.lookupTransform("/base_link", "/head_camera_depth_optical_frame", ros::Time(0), transform3);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("Failed to find transform betwenn /head_camera_depth_optical_frame and /base_link: %s",
                      ex.what());
        }*/

        //select stack type
        //setDefaultDrop(run, transform2.getOrigin().z() + .18, &g, &dropLocation);
        //stackDropLocation(run, transform2.getOrigin().z() + .18, &g, &dropLocation);
        //stackDropLocationOffset(run, transform2.getOrigin().z() + .18, &g, &dropLocation);
        //stackDropLocation(run, .81 + .18, &g, &dropLocation);
        //crop field for octomap

       // tf::TransformListener listener;

        //defaultDropPerception(&g, &dropLocation);
        stackDropPerception(&g, &dropLocation, transform2.getOrigin().z() + .18);


        group.setPoseTarget(offset(.1, dropLocation, g));

        group.plan(movement_plan);
        visual_tools.prompt("Press 'next' to start moement");
        group.execute(movement_plan);
        ROS_INFO("Moving above drop location");



        //move down
        group.setPoseTarget(dropLocation);

        group.plan(movement_plan);
        visual_tools.prompt("Press 'next' to start moement");
        group.execute(movement_plan);
        ROS_INFO("Moving to drop location");

        //drop object

        visual_tools.prompt("Press 'next' to drop object");
        ROS_INFO("opening grip");
        goal.command.position = .060;
        goal.command.max_effort = 150;
        ac.sendGoal(goal);
        sleep(5);

        group.setPoseTarget(offset(.1, dropLocation, g));

        group.plan(movement_plan);
        visual_tools.prompt("Press 'next' to start moement");
        group.execute(movement_plan);
        ROS_INFO("moving up");

        //move back to orig

        group.setJointValueTarget(group_variable_orig_values);

        group.plan(movement_plan);
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
        ROS_INFO("Moving back to original position");
        group.execute(movement_plan);


        //clear octomap
        ROS_INFO_STREAM("wait service /clear_octomap...");
        ros::service::waitForService("/clear_octomap");
        ROS_INFO_STREAM("... done");
        ros::ServiceClient clear_octomap_srv = nh.serviceClient<std_srvs::Empty>("/clear_octomap");
        std_srvs::Empty srv;
        auto result = clear_octomap_srv.call(srv);
        ROS_INFO_STREAM("calling /clear_octomap...");
        ROS_INFO_STREAM(" " << std::boolalpha << result);

        //reset head pos
        ROS_INFO("Moving to Center");

        head_goal.max_velocity = 1;
        head_goal.target.point.x = .7;
        head_goal.target.point.y = 0;
        head_goal.target.point.z = .35555;
        head_goal.target.header.frame_id = "base_link";

        hm.sendGoal(head_goal);
        hm.sendGoal(head_goal);

        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to restart");

        run++;
    }


    ROS_INFO("Exited after %d runs", run--);

}

//apply offset to geometry_msg Pose of amount amt, in the Z (verticle) direction, used when moving up/down toward block
geometry_msgs::Pose offset(double amt, geometry_msgs::Pose position, tf::Quaternion g){
    //create return variable
    geometry_msgs::Pose off_drop;

    //fill parameters with passed values (pos and orientation), and add amt to z
    off_drop.position.x = position.position.x;
    off_drop.position.y = position.position.y;
    off_drop.position.z = position.position.z + amt;
    off_drop.orientation.x = g.x();
    off_drop.orientation.y = g.y();
    off_drop.orientation.z = g.z();
    off_drop.orientation.w = g.w();

    return off_drop;
}

//stack block one on top of each other, uses run for height, no perception
void setDefaultDrop(int run, double height, tf::Quaternion* g, geometry_msgs::Pose* dropLocation)
{
    //set g to default rotation
    g->setEulerZYX(1.57,1.57,0);

    //set values to defailt location, + .035*run for height offset
    dropLocation->position.x = .45;
    dropLocation->position.y = -.22 -.3;
    dropLocation->position.z = height + .035*run + .004;
    dropLocation->orientation.x = g->x();
    dropLocation->orientation.y = g->y();
    dropLocation->orientation.z = g->z();
    dropLocation->orientation.w = g->w();

    return;
}

//2x2 tower, stack around given point and angle.  No perception, uses run number for height and orientation
void stackDropLocation(int run, double height, tf::Quaternion* g, geometry_msgs::Pose* dropLocation)
{
    //variables to hold offset from default values
    double x = 0;
    double y = 0;
    double z = 0;
    if(((run/2)%2)==0){
        //add y offset of block facing in x direction, and set angle
        y= .037;
        g->setEulerZYX(0,1.57,0);
    }
    else{
        //add x offfset if block is facing in y direction, and set angle
        x=-.037;
        g->setEulerZYX(-1.57,1.57,0);
    }
    if((run%2)==0){
        //change to negative if needed
        x = x*-1;
        y = y*-1;
    }
    //incr changed from .035 to .0325
    z = (run/2)*.035;

    //set default location with x/y/z offset, and angle
    dropLocation->position.x = .45+x;
    dropLocation->position.y = -.22 -.3+y;
    dropLocation->position.z = height + z + .004;
    dropLocation->orientation.x = g->x();
    dropLocation->orientation.y = g->y();
    dropLocation->orientation.z = g->z();
    dropLocation->orientation.w = g->w();

    return;
}

//set target location for grasping block on table, target is at block itself
void setTarget(tf::Quaternion q, geometry_msgs::Pose* target, tf::StampedTransform transform)
{
    //fill location target values with passed values
    target->position.x = transform.getOrigin().x();
    target->position.y = transform.getOrigin().y();
    target->position.z = transform.getOrigin().z() + .4;
    target->orientation.x = q.x();
    target->orientation.y = q.y();
    target->orientation.z = q.z();
    target->orientation.w = q.w();

    return;
}

//varient of stack drop that adds an offset to adjust for gripper grabbing closer to one end of a block
void stackDropLocationOffset(int run, double height, tf::Quaternion* g, geometry_msgs::Pose* dropLocation)
{
    //away from robot and to thr right for normalization .0055
    //variables to hold offset from default values
    double x = 0;
    double y = 0;
    double z = 0;
    if(((run/2)%2)==0){
        //add y offset of block facing in x direction, and set angle
        y= .037;
        g->setEulerZYX(0,1.57,0);
    }
    else{
        //add x offset of block facing in x direction, and set angle
        x=-.037;
        g->setEulerZYX(-1.57,1.57,0);
    }
    if((run%2)==0){
        //change to negative if needed
        x = x*-1;
        y = y*-1;
    }
    //add offsets of .0065 to variable that was not changed yet, to adjust for gripper not grasping center
    if(y!= 0){
        x = .0065;
    }
    else
        y = .0065;
    //incr changed from .035 to .0325
    z = (run/2)*.035;

    //set default location with x/y/z offset, and angle
    dropLocation->position.x = .45+x;
    dropLocation->position.y = -.22 -.3+y;
    dropLocation->position.z = height + z + .004;
    dropLocation->orientation.x = g->x();
    dropLocation->orientation.y = g->y();
    dropLocation->orientation.z = g->z();
    dropLocation->orientation.w = g->w();

    return;
}

//defailt drop varient, that can detect highest block in certain area and target that block for dropping
//also detects angle to work even if block is rotated
void defaultDropPerception(tf::Quaternion* g, geometry_msgs::Pose* dropLocation)
{
    //set default drop angle
    g->setEulerZYX(1.57,1.57,0);
    sensor_msgs::PointCloud2 tempCloud;
    tf::TransformListener listener2;
    //transform cloud_filtered_two into base_link reference frame, and store as tempCloud
    pcl_ros::transformPointCloud("base_link", cloud_filtered_two, tempCloud, listener2);

    //variables to hold clouds for use
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

    //transform sensor_msgs tempCloud to pcl cloud_f
    pcl::fromROSMsg(tempCloud, *cloud_f);

    //passthrough filters to remove everything outisde specified area
    //perception will only detect objects in this area
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_f);
    pass.setFilterFieldName ("y");
    //pass.setFilterLimits (-.05+-.11, .05+-.11);
    pass.setFilterLimits (-.6-.15,-.6+.15);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_p);

    pass.setInputCloud (cloud_p);
    pass.setFilterFieldName ("x");
    //pass.setFilterLimits (-.05 + -.19, .05 + -.19);
    pass.setFilterLimits (.5-.15, .5+ .15);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_p);

    //pub2.publish(cloud_p);

    //find highest point
    double maxz = cloud_p->points[0].z;
    for (int p = 0; p < cloud_p->points.size(); ++p) {
        if(cloud_p->points[p].z > maxz){
            maxz = cloud_p->points[p].z;
        }
    }

    //remove everything more than .01 m away from highest point
    //leaves only the top 1/3 of the highest block
    pass.setInputCloud (cloud_p);
    pass.setFilterFieldName ("z");
    //pass.setFilterLimits (-.05 + -.19, .05 + -.19);
    pass.setFilterLimits ( maxz-.01, maxz);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_f);

    //pub2.publish(cloud_f);

    //populate kd tree for cluster identification
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_f);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.01); // 2cm
    ec.setMinClusterSize (1);
    ec.setMaxClusterSize (350);
    //ec.setMinClusterSize (2800);
    //ec.setMaxClusterSize (35000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_f);
    ec.extract (cluster_indices);

    //bool used to determine if default drop must be used
    bool success = true;
    //int j, incremented through each loop, used to increase number at end of tf frame name, so each has unique name
    std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        //loop unil no new cluster found
        //creat pcl object to store found cloud temporarily
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud <pcl::PointXYZ>);
        for (const auto &idx : it->indices)
            cloud_cluster->push_back((*cloud_f)[idx]);
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        //print number of points in cloud
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        //create string to name cloud, using j to have incrementing number

        //find average x/y/z values, for target location, find maxy, miny, and minx points for rotation identification
        int num_points = cloud_cluster->size();
        double x = 0;
        double y = 0;
        double z = 0;
        //double maxx = cloud_cluster->points[0].x;
        double maxy = cloud_cluster->points[0].y;
        double maxyX = cloud_cluster->points[0].x;
        double minx = cloud_cluster->points[0].x;
        double minxY = cloud_cluster->points[0].y;
        double miny = cloud_cluster->points[0].y;
        double minyX = cloud_cluster->points[0].x;
        //add all x/y/z values in cluster
        for (int p = 0; p < cloud_cluster->points.size(); ++p) {
            x = x + cloud_cluster->points[p].x;
            y = y + cloud_cluster->points[p].y;
            z = z + cloud_cluster->points[p].z;
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
        //.29 is width of table area cut out, if length is this large, than the table was detected, and not a block
        if((maxy-miny) >=.29){
            success = false;
            break;
        }
        //find average for x/y/z
        y = y / num_points;
        z = z / num_points;
        x = x / num_points;
        double result = 0;
        double slope;
        double lengthOne = sqrt((minyX - minx) * (minyX - minx) + (miny-minxY) * (miny-minxY));
        double  lengthTwo = sqrt((maxyX - minx) * (maxyX - minx) + (maxy-minxY) * (maxy-minxY));
       // ROS_ERROR("LengthOne = %.2lf\nLengthTwo = %.2lf", lengthOne, lengthTwo);
        if(lengthTwo < lengthOne){
            //slope = (miny-minxY)/(minyX - minx);
            slope = -1*(minyX - minx)/(miny-minxY);
            //ROS_INFO("%.2lf %.2lf\n%.2lf %.2lf", minyX, minx, miny, minxY);
            //ROS_INFO("Slope1 = %.2lf", slope);
        }
        else{
            //slope = (maxy-minxY)/(maxyX - minx);
            slope = -1*(maxyX - minx)/(maxy-minxY);
            //ROS_INFO("%.2lf %.2lf\n%.2lf %.2lf", maxyX, minx, maxy, minxY);
            //ROS_INFO("Slope2 = %.2lf", slope);
        }
        result = atan(slope);

        //test if result is a valid number, if true, set rotation, else, set success to false
        if(!isnan(result)){
            g->setEulerZYX(result - 1.57,1.57,0);
        }
        else{
            success = false;
            break;
        }

        //set values
        dropLocation->position.x = x;
        dropLocation->position.y = y;
        dropLocation->position.z = maxz+.215;
        dropLocation->orientation.x = g->x();
        dropLocation->orientation.y = g->y();
        dropLocation->orientation.z = g->z();
        dropLocation->orientation.w = g->w();
        break;
    }

    //if success==false, then the process failed, and default values are set to override the previous results
    if(!success){
        ROS_INFO("No Block Detected, commencing default drop");
        g->setEulerZYX(0, 1.57, 0);

        dropLocation->position.x = .45;
        dropLocation->position.y = -.6;
        dropLocation->position.z = maxz+.215;
        dropLocation->orientation.x = g->x();
        dropLocation->orientation.y = g->y();
        dropLocation->orientation.z = g->z();
        dropLocation->orientation.w = g->w();
    }
    return;
}

//varient of stackDropLocation that uses perception to determine drop location
void stackDropPerception(tf::Quaternion* g, geometry_msgs::Pose* dropLocation, double height)
{

    g->setEulerZYX(1.57,1.57,0);
    //convert sensor_msgs cloud_filtered_two to base_link, then to pcl_cloud cloud_f
    sensor_msgs::PointCloud2 tempCloud;
    tf::TransformListener listener2;
    pcl_ros::transformPointCloud("base_link", cloud_filtered_two, tempCloud, listener2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(tempCloud, *cloud_f);

    //crop out everything outside certain area, where perception will be done
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_f);
    pass.setFilterFieldName ("y");
    //pass.setFilterLimits (-.05+-.11, .05+-.11);
    pass.setFilterLimits (-.6-.15,-.6+.15);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_p);

    pass.setInputCloud (cloud_p);
    pass.setFilterFieldName ("x");
    //pass.setFilterLimits (-.05 + -.19, .05 + -.19);
    pass.setFilterLimits (.5-.15, .5+ .15);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_p);

    //pub2.publish(cloud_p);

    //find highest point
    double maxz = cloud_p->points[0].z;
    for (int p = 0; p < cloud_p->points.size(); ++p) {
        if(cloud_p->points[p].z > maxz){
            maxz = cloud_p->points[p].z;
        }
    }

    //crop everything below maxz out, leaving only the top 1/3 of the highest block(s)
    pass.setInputCloud (cloud_p);
    pass.setFilterFieldName ("z");
    //pass.setFilterLimits (-.05 + -.19, .05 + -.19);
    pass.setFilterLimits ( maxz-.01, maxz);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_f);

    //pub2.publish(cloud_f);

    //setup cluster identification
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_f);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (1);
    ec.setMaxClusterSize (350);
    //ec.setMinClusterSize (2800);
    //ec.setMaxClusterSize (35000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_f);
    ec.extract (cluster_indices);


    //int j, incremented through each loop, used to increase number each loop, counts the number of clusters identified
    int j = 0;
    double result;
    tf::Vector3 point;
    std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        //loop unil no new cluster found
        //creat pcl object to store found cloud temporarily
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud <pcl::PointXYZ>);
        for (const auto &idx : it->indices)
            cloud_cluster->push_back((*cloud_f)[idx]);
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        //print number of points in cloud
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        //create string to name cloud, using j to have incrementing number

        //get average x/y/z values, and maxy, minx, and miny points for rotation identification
        int num_points = cloud_cluster->size();
        double x = 0;
        double y = 0;
        double z = 0;
        //double maxx = cloud_cluster->points[0].x;
        double maxy = cloud_cluster->points[0].y;
        double maxyX = cloud_cluster->points[0].x;
        double minx = cloud_cluster->points[0].x;
        double minxY = cloud_cluster->points[0].y;
        double miny = cloud_cluster->points[0].y;
        double minyX = cloud_cluster->points[0].x;
        //add all x/y/z values in cluster

        for (int p = 0; p < cloud_cluster->points.size(); ++p) {
            x = x + cloud_cluster->points[p].x;
            y = y + cloud_cluster->points[p].y;
            z = z + cloud_cluster->points[p].z;
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
        result = 0;
        double slope;
        double lengthOne = sqrt((minyX - minx) * (minyX - minx) + (miny-minxY) * (miny-minxY));
        double  lengthTwo = sqrt((maxyX - minx) * (maxyX - minx) + (maxy-minxY) * (maxy-minxY));
        ROS_ERROR("LengthOne = %.2lf\nLengthTwo = %.2lf", lengthOne, lengthTwo);
        if(lengthTwo < lengthOne){
            //slope = (miny-minxY)/(minyX - minx);
            slope = -1*(minyX - minx)/(miny-minxY);
            ROS_INFO("%.2lf %.2lf\n%.2lf %.2lf", minyX, minx, miny, minxY);
            ROS_INFO("Slope1 = %.2lf", slope);
        }
        else{
            //slope = (maxy-minxY)/(maxyX - minx);
            slope = -1*(maxyX - minx)/(maxy-minxY);
            ROS_INFO("%.2lf %.2lf\n%.2lf %.2lf", maxyX, minx, maxy, minxY);
            ROS_INFO("Slope2 = %.2lf", slope);
        }
        result = (slope);

        tf::Vector3 point2(x, y, z);
        point = point2;

        //increment j
        j++;

    }



    ROS_INFO("clouds: %d", j);


    //if j==1, one block at highest level and the new block must be placed next to it
    //if j==2, two blocks detected at highest level and new block must be placed on the level above it
    //if j==0, then no block detected (table is too large for the cluster identification settings), use default location
    if(j==1){

        ROS_INFO("One block detected, getting lower cloud");

        //get pcl cloud including top block and cloud one block lower, used to determine if drop location
        //within boundaries of lower level
        pass.setInputCloud (cloud_p);
        pass.setFilterFieldName ("z");
        //pass.setFilterLimits (-.05 + -.19, .05 + -.19);
        pass.setFilterLimits ( maxz-.045, maxz);
        pass.setFilterLimitsNegative (false);
        pass.filter (*cloud_f2);
        //pub3.publish(cloud_f2);

        //find max/min x/y values for new cloud, used to determine if drop location is within bounds of lower level
        double insideTopX = cloud_f2->points[0].x;
        double insideBotX = cloud_f2->points[0].x;
        double insideTopY = cloud_f2->points[0].y;
        double insideBotY = cloud_f2->points[0].y;
        for (int p = 0; p < cloud_f2->points.size(); ++p) {
            if(cloud_f2->points[p].x < insideBotX){
                insideBotX = cloud_f2->points[p].x;
            }
            else if(cloud_f2->points[p].x > insideTopX){
                insideTopX = cloud_f2->points[p].x;
            }
            if(cloud_f2->points[p].y < insideBotY){
                insideBotY = cloud_f2->points[p].y;
            }
            else if(cloud_f2->points[p].y > insideTopY){
                insideTopY = cloud_f2->points[p].y;
            }
        }

        //setup rotation

        //test if result (calculated earlier) is a real number after atan
        if(!isnan(atan(result))){
            g->setEulerZYX(atan(result) + 1.57,1.57,0);
        }
        else{
            //if not a number, print warning
            ROS_WARN("nan");
        }

        //xmod/ymod/zmod is modified point found earlier for drop location
        //use cos(result) to get new drop location at angle off from the block
        double xmod = point.getX() + (cos(result)*.088);
        double ymod = point.getY() + (sin(result)*.088);
        double zmod = maxz + .215;

        //test if new point within boundaries
        if(!(xmod > insideBotX && xmod < insideTopX && ymod > insideBotY && ymod < insideTopY)){
            //if not within boundaries, subtract instead of add cos/sin to get other side of block
            ROS_WARN("Outside of valid area, swapping...");
            xmod = point.getX() - (cos(result)*.088);
            ymod = point.getY() - (sin(result)*.088);
        }

        //set drop location and return
        dropLocation->position.x = xmod;
        dropLocation->position.y = ymod;
        dropLocation->position.z = maxz+.215-.035;
        dropLocation->orientation.x = g->x();
        dropLocation->orientation.y = g->y();
        dropLocation->orientation.z = g->z();
        dropLocation->orientation.w = g->w();

        return;
    }
    else if(j>1){
        //if multiple clouds detected, place at next level
        ROS_INFO("Multiple clouds detected at highest level... \nAdding new block to next level");

        //check if result is valid number after tangent
        if(!isnan(atan(result))){
            g->setEulerZYX(atan(result),1.57,0);
        }
        else{

            ROS_WARN("nan");
        }

        //find average x/y values to get center of highest layer
        double insideTopX = 0;
        double insideBotY = 0;
        int num = 0;
        for (int p = 0; p < cloud_f->points.size(); ++p) {
            insideTopX = insideTopX + cloud_f->points[p].x;
            insideBotY = insideBotY + cloud_f->points[p].y;
            num++;
        }
        double xav = insideTopX/num;
        double yav = insideBotY/num;

        //modify average by sin/cos using result to offset to edge of layer
        double xmod = xav + (sin(result)*.048);
        double ymod = yav + (cos(result)*.048);
        double zmod = maxz;

        //set drop location for next level and return
        dropLocation->position.x = xmod;
        dropLocation->position.y = ymod;
        dropLocation->position.z = maxz+.215;
        dropLocation->orientation.x = g->x();
        dropLocation->orientation.y = g->y();
        dropLocation->orientation.z = g->z();
        dropLocation->orientation.w = g->w();

        return;

    }
    else{
        //if nothing found, resort to default
        //if default gets used improperly, collision avoidance shpould prevent the robot from knocking the tower over
        // if the robot tries to place inside the tower, execution will fail and the robot will drop the block back on the
        //table and return to start pos
        ROS_INFO("No blocks detected, placing at default location");

        //populate default values for g/drop location
        g->setEulerZYX(1.57,1.57,0);

        dropLocation->position.x = .5;
        dropLocation->position.y = -.5;
        dropLocation->position.z = height + .004;
        dropLocation->orientation.x = g->x();
        dropLocation->orientation.y = g->y();
        dropLocation->orientation.z = g->z();
        dropLocation->orientation.w = g->w();
    }
    return;
}