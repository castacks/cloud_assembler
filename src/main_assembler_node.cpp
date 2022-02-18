#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "file_io.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/convert.h>

#include <math.h>

using namespace visualization_msgs;

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;

PointCloudPtr map_cloud(new PointCloud);

double downsample_resolution;
double marker_scale = 1;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

PointCloudPtr loadPointCloud(std::string input_filename)
{
    // Load point cloud in PLY or PCD format.
    PointCloudPtr cloud(new PointCloud);
    std::string extension = boost::filesystem::extension(input_filename);
    if(extension == ".ply") 
    {
        if(pcl::io::loadPLYFile(std::string(input_filename), *cloud) != 0) 
        {
            std::cerr << "Failed to read: " << input_filename << std::endl;
            EXIT_FAILURE;
        }
    } 
    else if (extension == ".pcd") 
    {
        if(pcl::io::loadPCDFile(std::string(input_filename), *cloud) != 0) 
        {
            std::cerr << "Failed to read: " << input_filename << std::endl;
            EXIT_FAILURE;
        }
    } 
    else 
    {
        ROS_ERROR("Unknown input cloud type.");
        EXIT_FAILURE;
    }

  return cloud;
}

PointCloudPtr downsamplePointCloud(const PointCloudPtr input_cloud, const double resolution)
{
    PointCloudPtr sparse_cloud(new PointCloud);
    // ROS_INFO("in filter function before filter %lu points.", input_cloud->size());
    // ROS_INFO_STREAM("resolution is " << resolution);

    pcl::VoxelGrid<PointT> vgf;
    vgf.setInputCloud (input_cloud);
    vgf.setLeafSize (resolution, resolution, resolution);
    vgf.filter(*sparse_cloud);
    // RcOS_INFO("in filter function after filter %lu points.", sparse_cloud->size());

    PointCloudPtr crop_cloud(new PointCloud);
    for (auto &p : *sparse_cloud) 
    {
        double range = p.x * p.x + p.y * p.y + p.z * p.z;
        if (range >= 1.0 * 1.0) 
        {
            crop_cloud->points.push_back(std::move(p));
        }
    }
    return crop_cloud;
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::ostringstream s;
    std::ostringstream mouse_point_ss;

    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        // ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
        break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        // ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
        ROS_INFO_STREAM("Finishing Assembly");
        break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:

        break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        // ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
        break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        // ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
        break;
    }
    server->applyChanges();
}

Marker makeBox(InteractiveMarker &msg)
{
    Marker marker;
    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.25;
    marker.color.g = 0.9;
    marker.color.b = 0.1;
    marker.color.a = 1.0;
    return marker;
}

void makeMenuMarker( const tf::Vector3& position )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 6;

    int_marker.name = "Finish assembly";
    int_marker.description = "Finish assembly\n(Right Click)";

    InteractiveMarkerControl control;

    control.interaction_mode = InteractiveMarkerControl::MENU;
    control.name = "menu_only_control";

    Marker marker = makeBox(int_marker);
    control.markers.push_back(marker);
    control.always_visible = true;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    menu_handler.apply(*server, int_marker.name);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main_assembler_node");
    ros::NodeHandle n;
    ROS_INFO_STREAM("launching main assembler");

    ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_pcl", 0);
    ros::Publisher filtered_map_pub = n.advertise<sensor_msgs::PointCloud2>("/filtered_map_pcl", 0);

    std::string map_filename;
    if (!n.getParam("main_assembler/map_filename", map_filename)) 
    {
        ROS_ERROR("Can not find map_filename param.");
        return -1;
    }
    if (!n.getParam("main_assembler/downsample_resolution", downsample_resolution)) 
    {
        ROS_ERROR("Can not find downsample_resolution param.");
        return -1;
    }

    std::string overall_pcl_filename;
    if (!n.getParam("main_assembler/overall_pcl_filename", overall_pcl_filename)) 
    {
        ROS_ERROR("Can not find overall_pcl_filename param.");
        return -1;
    }
    if (!n.getParam("main_assembler/marker_scale", marker_scale)) 
    {
        ROS_ERROR("Can not find marker_scale param.");
        return -1;
    }

    map_cloud = loadPointCloud(map_filename);

    server.reset(new interactive_markers::InteractiveMarkerServer("basic_controls","",false));
    ros::Duration(0.1).sleep();
    menu_handler.insert( "Finish Assembly", &processFeedback);

    // sensor_msgs::PointCloud2Ptr map_msg(new sensor_msgs::PointCloud2);
    // pcl::toROSMsg(*map_cloud, *map_msg);
    // map_msg->header.frame_id = "map";

    PointCloudPtr filtered_map_cloud = downsamplePointCloud(loadPointCloud(map_filename), downsample_resolution);
    sensor_msgs::PointCloud2Ptr filtered_map_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*filtered_map_cloud, *filtered_map_msg);
    filtered_map_msg->header.frame_id = "map";

    tf::Vector3 position;
    position = tf::Vector3(0,0,0);
    makeMenuMarker(position);

    server->applyChanges();

    ros::Rate r(5);
    while(ros::ok())
    {
        // map_msg->header.stamp = ros::Time::now();
        // map_pub.publish(map_msg);
        filtered_map_msg->header.stamp = ros::Time::now();
        filtered_map_pub.publish(filtered_map_msg);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}