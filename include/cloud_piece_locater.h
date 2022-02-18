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

class CloudPieceLocator
{
public:
    CloudPieceLocator(const boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server, const ros::NodeHandle& n;);
    ~CloudPieceLocator();
    ros::Publisher

    
private:
    std::shared_ptr<ros::NodeHandle> n_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    interactive_markers::MenuHandler menu_handler_;
};