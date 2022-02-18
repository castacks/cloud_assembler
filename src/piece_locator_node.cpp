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
PointCloudPtr filtered_map_cloud(new PointCloud);
PointCloudPtr piece_cloud(new PointCloud);
PointCloudPtr adjusted_piece_cloud(new PointCloud);
PointCloudPtr filtered_piece_cloud(new PointCloud);

double maxcorr;
double downsample_resolution;
double marker_scale = 1;

double current_pos_x = 0.0;
double current_pos_y = 0.0;
double current_pos_z = 0.0;
double current_ori_w = 1.0;
double current_ori_x = 0.0;
double current_ori_y = 0.0;
double current_ori_z = 0.0;

tf::Transform piece_pose;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

class Localizer
{
    public:
        Localizer(const PointCloudPtr map_ptr, const PointCloudPtr scan_ptr, const Eigen::Affine3d &initial_transform):
            map_ptr_(map_ptr),
            scan_ptr_(scan_ptr),
            initial_transform_(initial_transform) 
            {
                aligned_scan_ptr_ = PointCloudPtr(new PointCloud);
            }
        ~Localizer() {}

        Eigen::Affine3d localize() 
        {
            if (map_ptr_ == nullptr || scan_ptr_ == nullptr) {
                ROS_WARN("Localizer: input point cloud are null pointers.");
                return Eigen::Affine3d::Identity();
            }
            // GICP - do we need to check for convergence?
            pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
            std::cout << "maxcorr is " << maxcorr << '\n';
            gicp.setMaxCorrespondenceDistance(maxcorr);
            gicp.setMaximumIterations(25);
            gicp.setInputTarget(map_ptr_);
            gicp.setInputSource(scan_ptr_);
            // initial_transform_.matrix().block<3,1>(0,3) = Eigen::Vector3d(0.05,0.05,-0.05);
            // std::cout << "manual offset \n" << initial_transform_.matrix();
            gicp.align(*aligned_scan_ptr_, initial_transform_.matrix().cast<float>());
            // std::cout << "after align: \n" << gicp.getFinalTransformation().template cast<double>();
            optimized_transform_.matrix() = gicp.getFinalTransformation().template cast<double>();
            if (gicp.hasConverged()) 
            {
                ROS_INFO("Autocalibration GICP succeeded!");
                return optimized_transform_;
            } 
            else 
            {
                ROS_WARN("Autocalibration GICP failed!");
                EXIT_FAILURE;
            }
        }
        PointCloudPtr getAlignedScan() 
        {
            return aligned_scan_ptr_;
        }
    private:
        PointCloudPtr map_ptr_;
        PointCloudPtr scan_ptr_;
        PointCloudPtr aligned_scan_ptr_;
        Eigen::Affine3d initial_transform_;
        Eigen::Affine3d optimized_transform_;
};

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

pcl::visualization::PCLVisualizer::Ptr visualizePointCloud(const PointCloudPtr map_cloud, const PointCloudPtr scan_cloud, const std::string viewer_name)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewer_name));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> map_color(map_cloud, 255, 0, 0); // red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scan_color(scan_cloud, 255, 255, 0); // yellow
    viewer->addPointCloud(map_cloud, map_color, "map");
    viewer->addPointCloud(scan_cloud, scan_color, "scan");
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scan");
    viewer->addCoordinateSystem(2.0);
    viewer->initCameraParameters();

    return viewer;
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
        {
            ROS_INFO_STREAM("Local Alignment Start");
            Eigen::Quaterniond initial_q(current_ori_w, current_ori_x, current_ori_y, current_ori_z);
            Eigen::Matrix4d initial_transform_matrix = Eigen::Matrix4d::Identity();
            initial_transform_matrix(0,3) = current_pos_x;
            initial_transform_matrix(1,3) = current_pos_y;
            initial_transform_matrix(2,3) = current_pos_z;
            initial_transform_matrix.block<3,3>(0,0) = initial_q.normalized().toRotationMatrix();
            Eigen::Affine3d initial_transform;
            initial_transform.matrix() = initial_transform_matrix;

            PointCloudPtr initial_aligned_piece_cloud(new PointCloud);
            pcl::transformPointCloud(*filtered_piece_cloud, *initial_aligned_piece_cloud, initial_transform_matrix);
            pcl::visualization::PCLVisualizer::Ptr initial_viewer = visualizePointCloud(filtered_map_cloud, initial_aligned_piece_cloud, "Initial Alignment");
            
            while(!initial_viewer->wasStopped())
            {
                initial_viewer->spinOnce(100);
                usleep(100000);
            }

            Localizer loc(filtered_map_cloud, filtered_piece_cloud, initial_transform);
            Eigen::Affine3d optimized_transform = loc.localize();
            *filtered_piece_cloud = *loc.getAlignedScan();
            std::cout << optimized_transform.matrix() << std::endl;

            geometry_msgs::Pose pose;
            pose.position.x = 0;
            pose.position.y = 0;
            pose.position.z = 0;
            server->setPose(feedback->marker_name, pose);
            server->applyChanges();
            current_pos_x = 0;
            current_pos_y = 0;
            current_pos_z = 0;
            current_ori_w = 1;
            current_ori_x = 0;
            current_ori_y = 0;
            current_ori_z = 0;
            break;
        }

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            current_pos_x = feedback->pose.position.x;
            current_pos_y = feedback->pose.position.y;
            current_pos_z = feedback->pose.position.z;
            current_ori_w = feedback->pose.orientation.w;
            current_ori_x = feedback->pose.orientation.x;
            current_ori_y = feedback->pose.orientation.y;
            current_ori_z = feedback->pose.orientation.z;
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
    marker.color.r = 0.9;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    marker.color.a = 1.0;
    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof)
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 2;

    int_marker.name = "assumed init position";
    int_marker.description = "assumed init position";

    // insert a box
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    if ( fixed )
    {
        // int_marker.name += "_fixed";
        // int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
        int_marker.name += "_" + mode_text;
        int_marker.description = std::string("Assumed Init Position") + "\n" + "Drag cube or use RGB markers to move";
    }

    if(show_6dof)
    {
        tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        menu_handler.apply(*server, int_marker.name);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "piece_locator");
    ros::NodeHandle n;

    std::string node_name;
    node_name = ros::this_node::getName();

    ROS_INFO_STREAM("launching piece locater No." << node_name.back());

    std::string map_filename;
    if (!n.getParam("main_assembler/map_filename", map_filename)) 
    {
        ROS_ERROR("Can not find map_filename param.");
        return -1;
    }
    
    std::string piece_filename;
    if (!n.getParam(node_name+"/piece_filename", piece_filename)) 
    {
        ROS_ERROR("Can not find piece_filename param.");
        return -1;
    }

    if (!n.getParam(node_name+"/downsample_resolution", downsample_resolution)) 
    {
        ROS_ERROR("Can not find downsample_resolution param.");
        return -1;
    }

    if (!n.getParam(node_name+"/MaxCorrespondenceDistance", maxcorr)) 
    {
        ROS_ERROR("Can not find MaxCorrespondenceDistance param.");
        return -1;
    }

    ROS_INFO_STREAM("piece locator complete");

    map_cloud = loadPointCloud(map_filename);
    filtered_map_cloud = downsamplePointCloud(loadPointCloud(map_filename), downsample_resolution);

    piece_cloud = loadPointCloud(piece_filename);
    filtered_piece_cloud = downsamplePointCloud(loadPointCloud(piece_filename), downsample_resolution); // initialize, it will be the located piece cloud later

    std::string filtered_piece_topic = node_name+"/filtered_piece";
    // ROS_INFO_STREAM("topic is " << filtered_piece_topic);
    ros::Publisher filtered_piece_pub = n.advertise<sensor_msgs::PointCloud2>(filtered_piece_topic, 0);

    server.reset(new interactive_markers::InteractiveMarkerServer(node_name,"",false));

    menu_handler.insert("Local Alignment", &processFeedback);

    tf::Vector3 position;
    position = tf::Vector3(0,0,0);
    make6DofMarker(true, visualization_msgs::InteractiveMarkerControl::MOVE_3D, position, true);

    server->applyChanges();

    ros::Rate r(10);
    while(ros::ok())
    {
        Eigen::Quaterniond q_curr(current_ori_w, current_ori_x, current_ori_y, current_ori_z);
        Eigen::Matrix4d t_map2piece = Eigen::Matrix4d::Identity();
        t_map2piece(0,3) = current_pos_x;
        t_map2piece(1,3) = current_pos_y;
        t_map2piece(2,3) = current_pos_z;
        t_map2piece.block<3,3>(0,0) = q_curr.normalized().toRotationMatrix();
        Eigen::Affine3d aff_map2piece;
        aff_map2piece.matrix() = t_map2piece;
        static tf::TransformBroadcaster br_map2piece;
        tf::transformEigenToTF (aff_map2piece, piece_pose);
        br_map2piece.sendTransform(tf::StampedTransform(piece_pose, ros::Time::now(), "map", node_name+"_frame"));

        sensor_msgs::PointCloud2Ptr filtered_piece_msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*filtered_piece_cloud, *filtered_piece_msg);
        filtered_piece_msg->header.frame_id = node_name+"_frame";
        filtered_piece_msg->header.stamp = ros::Time::now();
        filtered_piece_pub.publish(filtered_piece_msg);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}