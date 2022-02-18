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

size_t scanCount = 0;
PointCloudPtr scan_cloud(new PointCloud);
PointCloudPtr map_cloud(new PointCloud);
PointCloudPtr acc_scan_cloud(new PointCloud);

double current_pos_x = 0.0;
double current_pos_y = 0.0;
double current_pos_z = 0.0;
double current_ori_w = 1.0;
double current_ori_x = 0.0;
double current_ori_y = 0.0;
double current_ori_z = 0.0;
bool reloc_finish = false;
double maxcorr;
double downsample_resolution;

ros::Publisher transformed_scan_pub;
ros::Publisher map_pub;
tf::TransformListener* tf_listener_ptr;

tf::Transform fake_launch_pose;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
// %EndTag(vars)%

class Localizer
{
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
        gicp.setMaxCorrespondenceDistance(maxcorr);
        gicp.setMaximumIterations(25);
        gicp.setInputTarget(map_ptr_);
        gicp.setInputSource(scan_ptr_);
        // initial_transform_.matrix().block<3,1>(0,3) = Eigen::Vector3d(0.05,0.05,-0.05);
        // std::cout << "manual offset \n" << initial_transform_.matrix();
        gicp.align(*aligned_scan_ptr_, initial_transform_.matrix().cast<float>());
        // std::cout << "after align: \n" << gicp.getFinalTransformation().template cast<double>();
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

PointCloudPtr loadPointCloud(std::string input_filename)
{
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
  for (auto &p : *sparse_cloud) {
    double range = p.x * p.x + p.y * p.y + p.z * p.z;

    if (range >= 1.0 * 1.0) {
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

void scanCallback(const sensor_msgs::PointCloud2 &msg)
{
  PointCloud scan;
  pcl::fromROSMsg(msg, scan);

  // Eigen::Affine3d t_init2sensor;
  // t_init2sensor.matrix() = Eigen::Matrix4d::Identity();
  // tf::StampedTransform tf_init2sensor;
  // try
  // {
  //   tf_listener_ptr->lookupTransform("/sensor_init", "/sensor", msg.header.stamp, tf_init2sensor);
  // }
  // catch (tf::TransformException &ex)
  // {
  //   ROS_WARN("%s",ex.what());
  // }
  // tf::transformTFToEigen(tf_init2sensor, t_init2sensor);

  // Eigen::Matrix4d mat_init2sensor = t_init2sensor.matrix();
  // Eigen::Matrix4d t_map2sensor = t_map2init * mat_init2sensor;

  // PointCloudPtr transformed_scan (new PointCloud);
  // scan in sensor_init frame
  // pcl::transformPointCloud(scan, *transformed_scan, t_init2sensor);

  // accumulating transformed scan cloud in sensor_init
  acc_scan_cloud->insert(acc_scan_cloud->end(), scan.begin(), scan.end());
  PointCloudPtr filtered_acc_scan_cloud(new PointCloud);
  filtered_acc_scan_cloud = downsamplePointCloud(acc_scan_cloud, downsample_resolution);
  *acc_scan_cloud = *filtered_acc_scan_cloud;
  // ROS_INFO("Accumulated %lu points from laser after filtering.", acc_scan_cloud->size());
  // ROS_INFO_STREAM("sensor position is at:" << '\n' << t_init2sensor.matrix());
  scanCount++;

  // transform the accumulated cloud to map frame for interface relocalization
  Eigen::Quaterniond q_curr(current_ori_w, current_ori_x, current_ori_y, current_ori_z);
  Eigen::Matrix4d t_map2init = Eigen::Matrix4d::Identity();
  t_map2init(0,3) = current_pos_x;
  t_map2init(1,3) = current_pos_y;
  t_map2init(2,3) = current_pos_z;
  t_map2init.block<3,3>(0,0) = q_curr.normalized().toRotationMatrix();
  Eigen::Affine3d aff_map2init;
  aff_map2init.matrix() = t_map2init;
  // PointCloudPtr transformed_acc_scan_cloud(new PointCloud);
  // pcl::transformPointCloud(*acc_scan_cloud, *transformed_acc_scan_cloud, t_map2init);

  static tf::TransformBroadcaster br_map2init;
  tf::transformEigenToTF (aff_map2init, fake_launch_pose);
  br_map2init.sendTransform(tf::StampedTransform(fake_launch_pose, ros::Time::now(), "uav1/map", "sensor_init"));


  sensor_msgs::PointCloud2Ptr map_msg(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*acc_scan_cloud, *cloud_msg);
  pcl::toROSMsg(*map_cloud, *map_msg);
  cloud_msg->header.frame_id = "sensor_init";
  cloud_msg->header.stamp = ros::Time::now();
  map_msg->header.frame_id = "uav1/map";
  map_msg->header.stamp = ros::Time::now();
  transformed_scan_pub.publish(cloud_msg);
  map_pub.publish(map_msg);
}

void finishCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("start accumulating pointcloud");
  reloc_finish = true;
}
// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
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
// %EndTag(Box)%

// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "uav1/map", "uav1/map_pub"));

  counter++;
}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  // s << "Feedback from marker '" << feedback->marker_name << "' "
  //     << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    // mouse_point_ss << " at " << feedback->mouse_point.x
    //                << ", " << feedback->mouse_point.y
    //                << ", " << feedback->mouse_point.z
    //                << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      // ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      // ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      reloc_finish = true;
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      current_pos_x = feedback->pose.position.x;
      current_pos_y = feedback->pose.position.y;
      current_pos_z = feedback->pose.position.z;

      current_ori_w = feedback->pose.orientation.w;
      current_ori_x = feedback->pose.orientation.x;
      current_ori_y = feedback->pose.orientation.y;
      current_ori_z = feedback->pose.orientation.z;
      // ROS_INFO_STREAM( s.str() << ": pose changed"
      //     << "\nposition = "
      //     << feedback->pose.position.x
      //     << ", " << feedback->pose.position.y
      //     << ", " << feedback->pose.position.z
      //     << "\norientation = "
      //     << feedback->pose.orientation.w
      //     << ", " << feedback->pose.orientation.x
      //     << ", " << feedback->pose.orientation.y
      //     << ", " << feedback->pose.orientation.z
      //     << "\nframe: " << feedback->header.frame_id
      //     << " time: " << feedback->header.stamp.sec << "sec, "
      //     << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(alignMarker)%
void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  ROS_INFO_STREAM( feedback->marker_name << ":"
      << " aligning position = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << " to "
      << pose.position.x
      << ", " << pose.position.y
      << ", " << pose.position.z );

  server->setPose( feedback->marker_name, pose );
  server->applyChanges();
}
// %EndTag(alignMarker)%

double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}

////////////////////////////////////////////////////////////////////////////////////

// %Tag(6DOF)%
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "uav1/map";
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
    menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%

// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n;
  ROS_INFO_STREAM("launching");
  std::string map_filename;
  if (!n.getParam("interface_reloc/map_filename", map_filename)) {
      ROS_ERROR("Can not find map_filename param.");
      return -1;
  }

  if (!n.getParam("interface_reloc/downsample_resolution", downsample_resolution)) {
      ROS_ERROR("Can not find downsample_resolution param.");
      return -1;
  }

  if (!n.getParam("interface_reloc/MaxCorrespondenceDistance", maxcorr)) {
      ROS_ERROR("Can not find MaxCorrespondenceDistance param.");
      return -1;
  }
  std::string optimized_transform_filename;
  if (!n.getParam("interface_reloc/optimized_transform_filename", optimized_transform_filename)) {
      ROS_ERROR("Can not find optimized_transform_filename param.");
      return -1;
  }
  bool run_with_superodometry = false; // registered point instead of pure scan
  if (!n.getParam("interface_reloc/use_superodom", run_with_superodometry)) {
      ROS_ERROR("Can not find use_superodom param.");
      return -1;
  }

  std::string scan_to_process;
  if (run_with_superodometry)
  {
    ROS_INFO("using superodom result");
    scan_to_process = "/uav1/velodyne_cloud_registered_imu";
  }
  else
  {
    ROS_INFO("using local point overlay");
    scan_to_process = "/scan";
  }

  map_cloud = loadPointCloud(map_filename);
  PointCloudPtr filtered_map_cloud = downsamplePointCloud(loadPointCloud(map_filename), downsample_resolution);
  // ROS_INFO("Before filter %lu map points.", map_cloud->size());
  ROS_INFO("Load %lu map points.", filtered_map_cloud->size());

  transformed_scan_pub = n.advertise<sensor_msgs::PointCloud2>("/transformed_scan", 0);
  map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_pcl", 0);
  ros::Subscriber scan_sub = n.subscribe(scan_to_process, 2, scanCallback);
  ros::Subscriber finish_sub = n.subscribe("/finish", 2, finishCallback);
  tf::TransformListener tf_listener;
  tf_listener_ptr = &tf_listener;

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

  ros::Duration(0.1).sleep();

  menu_handler.insert( "Finish Alignment", &processFeedback );
  // menu_handler.insert( "Second Entry", &processFeedback );
  // interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
  // menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
  // menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );

  tf::Vector3 position;
  position = tf::Vector3(0, 0, 0);
  make6DofMarker( true, visualization_msgs::InteractiveMarkerControl::MOVE_3D, position, true );


  server->applyChanges();
  ros::Rate r_tfstatic(100);
  Eigen::Affine3d identity_tf;
  identity_tf.matrix() = Eigen::Matrix4d::Identity();
  tf::transformEigenToTF (identity_tf, fake_launch_pose);
  static tf::TransformBroadcaster fake_tf_broadcaster;
  while(!reloc_finish)
  {
    fake_tf_broadcaster.sendTransform(tf::StampedTransform(fake_launch_pose, ros::Time::now(), "uav1/map", "sensor_init"));
    r_tfstatic.sleep();
    ros::spinOnce();
  }
  // ros::spin();

  // PointCloudPtr filtered_scan_cloud = downsamplePointCloud(acc_scan_cloud, downsample_resolution);
  pcl::io::savePCDFileBinary ("/home/shimizu/relocalize_map.pcd", *acc_scan_cloud);
  PointCloudPtr filtered_scan_cloud(new PointCloud);
  *filtered_scan_cloud = *acc_scan_cloud;
  ROS_INFO("Before filter %lu scan points.", acc_scan_cloud->size());
  ROS_INFO("Load %lu scan points.", filtered_scan_cloud->size());
  PointCloudPtr initial_aligned_scan_cloud(new PointCloud);
  Eigen::Quaterniond initial_q(current_ori_w, current_ori_x, current_ori_y, current_ori_z);
  Eigen::Matrix4d initial_transform_matrix = Eigen::Matrix4d::Identity();
  initial_transform_matrix(0,3) = current_pos_x;
  initial_transform_matrix(1,3) = current_pos_y;
  initial_transform_matrix(2,3) = current_pos_z;
  initial_transform_matrix.block<3,3>(0,0) = initial_q.normalized().toRotationMatrix();
  pcl::transformPointCloud(*filtered_scan_cloud, *initial_aligned_scan_cloud, initial_transform_matrix);

  Eigen::Affine3d initial_transform;
  initial_transform.matrix() = initial_transform_matrix;
  std::cout << "initial transform:\n" << initial_transform.matrix() << std::endl;

  pcl::visualization::PCLVisualizer::Ptr initial_viewer = visualizePointCloud(filtered_map_cloud, initial_aligned_scan_cloud, "Initial Alignment");

  // visualization
  while(!initial_viewer->wasStopped ())
  {
      initial_viewer->spinOnce(100);
      usleep(100000);
  }

  ROS_INFO("Starting registration/localization.");
  Localizer loc(filtered_map_cloud, filtered_scan_cloud, initial_transform);
  Eigen::Affine3d optimized_transform = loc.localize();
  PointCloudPtr aligned_scan = loc.getAlignedScan();
  // PointCloudPtr final_aligned_scan_cloud(new PointCloud);
  // pcl::transformPointCloud(*filtered_scan_cloud, *final_aligned_scan_cloud, optimized_transform.matrix());
  // output transform
  std::cout << optimized_transform.matrix() << std::endl;
  std::ofstream file(optimized_transform_filename);
  file << optimized_transform.matrix() << std::endl;

  // pcl::io::savePCDFileBinary ("/home/shimizu/aligned.pcd", *aligned_scan);
  // pcl::io::savePCDFileBinary ("/home/shimizu/final_aligned.pcd", *final_aligned_scan_cloud);
  // pcl::io::savePCDFileBinary ("/home/shimizu/map.pcd", *filtered_map_cloud);
  // visualization
  pcl::visualization::PCLVisualizer::Ptr viewer = visualizePointCloud(filtered_map_cloud, aligned_scan, "Final Alignment");

  while(!viewer->wasStopped ())
  {
      viewer->spinOnce(100);
      usleep(100000);
  }

  server.reset();

  static tf::TransformBroadcaster tf_broadcaster;
  tf::Transform launch_pose;
  tf::transformEigenToTF (optimized_transform, launch_pose);
  ros::Rate r(100);
  while (ros::ok())
  {
    // ROS_INFO_STREAM("publishing tf from previous map to sensor_init");
    tf_broadcaster.sendTransform(tf::StampedTransform(launch_pose, ros::Time::now(), "uav1/map", "sensor_init"));
    r.sleep();
  }

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;


}
// %EndTag(main)%
