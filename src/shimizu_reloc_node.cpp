#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "file_io.hpp"

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;

size_t scanCount = 0;
PointCloudPtr scan_cloud(new PointCloud);

class Localizer {
  public:
    Localizer(const PointCloudPtr map_ptr, const PointCloudPtr scan_ptr, const Eigen::Affine3d &initial_transform):
        map_ptr_(map_ptr),
        scan_ptr_(scan_ptr),
        initial_transform_(initial_transform) {
            aligned_scan_ptr_ = PointCloudPtr(new PointCloud);
        }
    ~Localizer() {}

    Eigen::Affine3d localize() {

        if (map_ptr_ == nullptr || scan_ptr_ == nullptr) {
            ROS_WARN("Localizer: input point cloud are null pointers.");
            return Eigen::Affine3d::Identity();
        }

        // GICP - do we need to check for convergence?
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setMaxCorrespondenceDistance(1.0);
        gicp.setMaximumIterations(25);
        gicp.setInputTarget(map_ptr_);
        gicp.setInputSource(scan_ptr_);
        gicp.align(*aligned_scan_ptr_, initial_transform_.matrix().cast<float>());
        optimized_transform_.matrix() = gicp.getFinalTransformation().template cast<double>();

        if (gicp.hasConverged()) {
            ROS_INFO("Autocalibration GICP succeeded!");
            return optimized_transform_;
        } else {
            ROS_WARN("Autocalibration GICP failed!");
            EXIT_FAILURE;
        }
    }

    PointCloudPtr getAlignedScan() {
        return aligned_scan_ptr_;
    }

  private:
    PointCloudPtr map_ptr_;
    PointCloudPtr scan_ptr_;
    PointCloudPtr aligned_scan_ptr_;
    Eigen::Affine3d initial_transform_;
    Eigen::Affine3d optimized_transform_;
};

PointCloudPtr loadPointCloud(std::string input_filename) {
    // Load point cloud in PLY or PCD format.
    PointCloudPtr cloud(new PointCloud);
    std::string extension = boost::filesystem::extension(input_filename);
    if(extension == ".ply") {
        if(pcl::io::loadPLYFile(std::string(input_filename), *cloud) != 0) {
            std::cerr << "Failed to read: " << input_filename << std::endl;
            EXIT_FAILURE;
        }
    } else if (extension == ".pcd") {
        if(pcl::io::loadPCDFile(std::string(input_filename), *cloud) != 0) {
            std::cerr << "Failed to read: " << input_filename << std::endl;
            EXIT_FAILURE;
        }
    } else {
        ROS_ERROR("Unknown input cloud type.");
        EXIT_FAILURE;
    }

    return cloud;
}

// PointCloudPtr downsamplePointCloud(const PointCloudPtr input_cloud, const double resolution) {
//
//     PointCloudPtr sparse_cloud(new PointCloud);
//
//     pcl::VoxelGrid<PointT> vgf;
//     vgf.setInputCloud (input_cloud);
//     vgf.setLeafSize (resolution, resolution, resolution);
//     vgf.filter(*sparse_cloud);
//
//     return sparse_cloud;
// }

PointCloudPtr downsamplePointCloud(const PointCloudPtr input_cloud, const double resolution)
{
  PointCloudPtr sparse_cloud(new PointCloud);
  ROS_INFO("in filter function before filter %lu points.", input_cloud->size());
  ROS_INFO_STREAM("resolution is " << resolution);

  pcl::VoxelGrid<PointT> vgf;
  vgf.setInputCloud (input_cloud);
  vgf.setLeafSize (resolution, resolution, resolution);
  vgf.filter(*sparse_cloud);
  ROS_INFO("in filter function after filter %lu points.", sparse_cloud->size());
  return sparse_cloud;
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

void scanCallback(const sensor_msgs::PointCloud2 &msg) {
    if (scanCount >= 10) {
        ROS_INFO("Accumulated %lu points from laser.", scan_cloud->size());
        return;
    } else {
        // stack scans
        PointCloud scan;
        pcl::fromROSMsg(msg, scan);
        scan_cloud->insert(scan_cloud->end(), scan.begin(), scan.end());
        scanCount++;
    }
}


int main(int argc, char**argv) {

    ros::init(argc, argv,"shimizu_reloc_node");
	ros::NodeHandle nh("~");

    std::string map_filename, scan_filename, initial_transform_filename;
    std::string optimized_transform_filename;
    double initial_x, initial_y, initial_z, initial_roll, initial_pitch, initial_yaw;
    double downsample_resolution;

    if (!nh.getParam("downsample_resolution", downsample_resolution)) {
        ROS_ERROR("Can not find downsample_resolution param.");
        return -1;
    }
    if (!nh.getParam("map_filename", map_filename)) {
        ROS_ERROR("Can not find map_filename param.");
        return -1;
    }
    if (!nh.getParam("scan_filename", scan_filename)) {
        ROS_ERROR("Can not find map_filename param.");
        return -1;
    }
    if (!nh.getParam("initial_transform_filename", initial_transform_filename)) {
        ROS_ERROR("Can not find initial_transform_filename param.");
        return -1;
    }
    if (!nh.getParam("initial_x", initial_x)) {
        ROS_ERROR("Can not find initial_x param.");
        return -1;
    }
    if (!nh.getParam("initial_y", initial_y)) {
        ROS_ERROR("Can not find initial_y param.");
        return -1;
    }
    if (!nh.getParam("initial_z", initial_z)) {
        ROS_ERROR("Can not find initial_z param.");
        return -1;
    }
    if (!nh.getParam("initial_roll", initial_roll)) {
        ROS_ERROR("Can not find initial_roll param.");
        return -1;
    }
    if (!nh.getParam("initial_pitch", initial_pitch)) {
        ROS_ERROR("Can not find initial_pitch param.");
        return -1;
    }
    if (!nh.getParam("initial_yaw", initial_yaw)) {
        ROS_ERROR("Can not find initial_yaw param.");
        return -1;
    }
    if (!nh.getParam("optimized_transform_filename", optimized_transform_filename)) {
        ROS_ERROR("Can not find optimized_transform_filename param.");
        return -1;
    }

    // load initial transform
    Eigen::Matrix3d initial_rotation;
    initial_rotation = Eigen::AngleAxisd(initial_yaw,   Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(initial_pitch, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(initial_roll,  Eigen::Vector3d::UnitX());
    Eigen::Vector3d initial_translation(initial_x, initial_y, initial_z);
    Eigen::Matrix4d initial_transform_matrix;
    initial_transform_matrix << initial_rotation, initial_translation, 0.0,0.0,0.0,1.0;

    // readData(initial_transform_filename, 4, 4, ',', initial_transform_matrix);
    Eigen::Affine3d initial_transform;
    initial_transform.matrix() = initial_transform_matrix;
    std::cout << "initial transform:\n" << initial_transform.matrix() << std::endl;

    // read map
    PointCloudPtr map_cloud = downsamplePointCloud(loadPointCloud(map_filename), downsample_resolution);
    ROS_INFO("Load %lu map points.", map_cloud->size());

    // read current scan
    ros::Subscriber scan_sub = nh.subscribe("/scan", 2, scanCallback);
    // PointCloudPtr scan_cloud = downsamplePointCloud(loadPointCloud(scan_filename), downsample_resolution);
    // ROS_INFO("Load %lu scan points.", scan_cloud->size());

    // while (scanCount <= 4 && ros::ok()) {
    //     ros::spinOnce();
    // }

    // visualize initialization
    while(scanCount < 10)
    {
      ROS_INFO_STREAM("accumulating pointclouds");
      ros::spinOnce();
    }

    PointCloudPtr filtered_scan_cloud = downsamplePointCloud(scan_cloud, downsample_resolution);
    ROS_INFO("Before filter %lu scan points.", scan_cloud->size());
    ROS_INFO("Load %lu scan points.", filtered_scan_cloud->size());
    PointCloudPtr initial_aligned_scan_cloud(new PointCloud);
    pcl::transformPointCloud(*filtered_scan_cloud, *initial_aligned_scan_cloud, initial_transform_matrix);
    pcl::visualization::PCLVisualizer::Ptr initial_viewer = visualizePointCloud(map_cloud, initial_aligned_scan_cloud, "Initial Alignment");

    while(!initial_viewer->wasStopped ())
    {
        initial_viewer->spinOnce(100);
        usleep(100000);
    }

    // generalized ICP based localization
    ROS_INFO("Starting registration/localization.");
    Localizer loc(map_cloud, filtered_scan_cloud, initial_transform);
    Eigen::Affine3d optimized_transform = loc.localize();
    PointCloudPtr aligned_scan = loc.getAlignedScan();
    // output transform
    std::cout << optimized_transform.matrix() << std::endl;
    std::ofstream file(optimized_transform_filename);
    file << optimized_transform.matrix() << std::endl;

    // visualization
    // pcl::visualization::PCLVisualizer::Ptr viewer = visualizePointCloud(map_cloud, aligned_scan, "Final Alignment");
    pcl::visualization::PCLVisualizer::Ptr viewer = visualizePointCloud(map_cloud, aligned_scan, "Final Alignment");

    while(!viewer->wasStopped ())
    {
        viewer->spinOnce(100);
        usleep(100000);
    }

    return 0;
}
