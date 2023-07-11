// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/random_sample.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <std_msgs/Header.h>
#include <cmath>
#include <chrono>

class PointCloudProcessor
{
public:
  // PointCloudProcessor(ros::NodeHandle nh, ros::NodeHandle pnh);
  PointCloudProcessor(ros::NodeHandle nh);
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
  pcl::PointCloud<pcl::PointXYZ>::Ptr extractPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients);

private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher dpub_;

  pcl::RandomSample<pcl::PointXYZ> sampler_;
  pcl::SACSegmentation<pcl::PointXYZ> seg_;
  pcl::ExtractIndices<pcl::PointXYZ> extract_;
  std::vector<float> plane_coeff = {0.000258076, -0.953586, -0.30112, 0.208252};

  const double sample_size_ = 3072;
  const int max_iterations_ = 800;
  const double distance_threshold_ = 0.001;

  const std::string input_topic_ = "/sync/pcl_cam";
  const std::string output_topic_ = "/sync/pcl_ground";
  // const std::string doutput_topic_ = "/sync/pcl_dnsmp";
  const std::string err_output_topic_ = "/sync/pcl_err";
};


// PointCloudProcessor::PointCloudProcessor(ros::NodeHandle nh, ros::NodeHandle pnh)
//   : input_topic_{ pnh.param<std::string>("input_topic", "input") }
//   , output_topic_{ pnh.param<std::string>("output_topic", "output") }
//   , sample_size_{ pnh.param<double>("sample_size", 0.1) }
//   , max_iterations_{ pnh.param<int>("max_iterations", 1000) }
//   , distance_threshold_{ pnh.param<double>("distance_threshold", 0.01) }
// {
//   // Create a subscriber for the input point cloud
//   sub_ = nh.subscribe(input_topic_, 1, &PointCloudProcessor::cloudCallback, this);

//   // Create a publisher for the downsampled and segmented point cloud
//   pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(output_topic_, 1);

//   // Configure the random downsampling filter
//   sampler_.setSample(sample_size_);

//   // Configure the RANSAC plane segmentation algorithm
//   seg_.setOptimizeCoefficients(true);
//   seg_.setModelType(pcl::SACMODEL_PLANE);
//   seg_.setMethodType(pcl::SAC_RANSAC);
//   seg_.setMaxIterations(max_iterations_);
//   seg_.setDistanceThreshold(distance_threshold_);

//   // Configure the point cloud extraction algorithm
//   extract_.setNegative(false);
// }


PointCloudProcessor::PointCloudProcessor(ros::NodeHandle nh)
{
  // Create a subscriber for the input point cloud
  sub_ = nh.subscribe(input_topic_, 1, &PointCloudProcessor::cloudCallback, this);

  // Create a publisher for the downsampled and segmented point cloud
  pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(output_topic_, 1);
  // dpub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(doutput_topic_, 1);

}


void PointCloudProcessor::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  // Convert the ROS point cloud message to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);

  // Perform random downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  // Configure the random downsampling filter
  sampler_.setInputCloud(cloud);
  sampler_.setSample(sample_size_);
  sampler_.filter(*downsampled);

  // Publish the downsampled point cloud
  // sensor_msgs::PointCloud2 dsp_pcl;
  // pcl::toROSMsg(*downsampled, dsp_pcl);
  // dsp_pcl.header.frame_id = "camera_link";
  // this->dpub_.publish(dsp_pcl);
  
  /* ********************************************
  *********** Single Plane Segment ************
  ******************************************** */
  // Perform plane segmentation using RANSAC
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr segmented(new pcl::PointCloud<pcl::PointXYZ>);
  // Configure the RANSAC plane segmentation algorithm
  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_PLANE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(max_iterations_);
  seg_.setDistanceThreshold(distance_threshold_);
  // Specify Y orientation as the priority direction
  Eigen::Vector3f axis = Eigen::Vector3f::UnitY();
  // Define search range around desired normal
  double radian = 10/180 * M_PI; // radian
  seg_.setAxis(axis);
  seg_.setEpsAngle(radian);
  // Execute the RANSAC plane segmentation algorithm
  seg_.setInputCloud(downsampled);
  seg_.segment(*inliers, *coefficients);

  // std::cout << *coefficients << std::endl;
  // std::cout << abs(coefficients->values[1]) << std::endl;

  // 通过平面参数判断是否为地面，若不为地面，则保留最近一帧地面的平面参数，以获取平面附近点
  if (abs(coefficients->values[1]) > 0.90) {
    this->plane_coeff[0] = coefficients->values[0];
    this->plane_coeff[1] = coefficients->values[1];
    this->plane_coeff[2] = coefficients->values[2];
    this->plane_coeff[3] = coefficients->values[3];
    // std::cout << "coefficients update" << std::endl;
    
    // Extract the plane from the point cloud
    // Configure the point cloud extraction algorithm
    extract_.setInputCloud(downsampled);
    extract_.setIndices(inliers);
    extract_.setNegative(false);
    // Execute the point cloud extraction algorithm
    extract_.filter(*segmented);
  }
  else {
    coefficients->values[0] = this->plane_coeff[0];
    coefficients->values[1] = this->plane_coeff[1];
    coefficients->values[2] = this->plane_coeff[2];
    coefficients->values[3] = this->plane_coeff[3];
    std::cout << "coefficients reserved" << std::endl;
    segmented = extractPoints(downsampled, coefficients);
  }

  // Publish the segmented point cloud
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*segmented, output);
  output.header.frame_id = "camera_link";
  this->pub_.publish(output);

  /* **********************************************
  *********** Single Plane Segment End ************
  ********************************************** */


  // /* ********************************************
  // *********** Multiple Plane Segment ************
  // ******************************************** */
  // // Define desired plane orientation
  // Eigen::Vector3f desired_normal = Eigen::Vector3f::UnitY();  // y-axis

  // // Define search range around desired normal
  // double search_range = 10.0;  // degrees

  // // Define plane segmentation parameters
  // seg_.setOptimizeCoefficients(true);
  // seg_.setModelType(pcl::SACMODEL_PLANE);
  // seg_.setMethodType(pcl::SAC_RANSAC);
  // seg_.setDistanceThreshold(distance_threshold_);
  // seg_.setMaxIterations(max_iterations_);

  // // Perform plane segmentation with different normal vectors
  // double min_angle_diff = std::numeric_limits<double>::max();
  // pcl::PointIndices::Ptr best_inliers(new pcl::PointIndices);
  // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // pcl::ModelCoefficients::Ptr best_coefficients(new pcl::ModelCoefficients);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr segmented(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ransac(new pcl::PointCloud<pcl::PointXYZ>);
  // // Initialize pcl_ransac & angle
  // pcl_ransac = downsampled;
  // double angle;
  // // pcl::PointCloud<pcl::PointXYZ>::Ptr best_plane(new pcl::PointCloud<pcl::PointXYZ>);
  // for (angle = -search_range; angle <= search_range; angle += 1.0) {
  //   // Define normal vector for this iteration
  //   Eigen::Vector3f normal = desired_normal;
  //   normal = Eigen::AngleAxisf(angle * M_PI / 180.0, Eigen::Vector3f::UnitZ()) * normal;

  //   // Set normal vector for segmentation
  //   seg_.setAxis(normal);
  //   seg_.setEpsAngle(angle * M_PI / 180.0);

  //   // Perform plane segmentation
  //   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  //   seg_.setInputCloud(pcl_ransac);
  //   seg_.segment(*inliers, *coefficients);

  //   // // Create a new point cloud containing only the inliers (points belonging to the plane)
  //   // pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
  //   // pcl::ExtractIndices<pcl::PointXYZ> extract;
  //   // extract.setInputCloud(cloud);
  //   // extract.setIndices(inliers);
  //   // extract.filter(*plane);

  //   // Compute angle difference between desired and estimated plane normal
  //   double angle_diff = std::acos(desired_normal.dot(Eigen::Vector3f(coefficients->values[0],
  //                                                                    coefficients->values[1],
  //                                                                    coefficients->values[2])) /
  //                                 (desired_normal.norm() * Eigen::Vector3f(coefficients->values[0],
  //                                                                          coefficients->values[1],
  //                                                                          coefficients->values[2]).norm()));
    
  //   if (angle_diff >= min_angle_diff) {
  //     extract_.setInputCloud(pcl_ransac);
  //     extract_.setIndices(inliers);
  //     extract_.setNegative(true);
  //     extract_.filter(*pcl_ransac);
  //     std::cout << "Surface Removed!" << std::endl;
  //   }
  //   else {
  //     min_angle_diff = angle_diff;
  //     // best_plane = plane;
  //     *best_inliers = *inliers;
  //     *best_coefficients = *coefficients;
  //     std::cout << "best plane update!!" << std::endl;
  //   }

  //   // if (angle_diff >= min_angle_diff) {
  //   //   std::cout << "angle_diff so big!!!" << std::endl;
  //   // }
  //   // else {
  //   //   min_angle_diff = angle_diff;
  //   //   // best_plane = plane;
  //   //   *best_inliers = *inliers;
  //   //   *best_coefficients = *coefficients;
  //   //   std::cout << "best plane update!!" << std::endl;
  //   // }
  // }

  // // std::cout << *best_coefficients << std::endl;
  // // std::cout << abs(best_coefficients->values[1]) << std::endl;
  
  // // 通过平面参数判断是否为地面，若不为地面，则保留最近一帧地面的平面参数，以获取平面附近点
  // if (abs(best_coefficients->values[1]) > 0.90) {
  //   this->plane_coeff[0] = best_coefficients->values[0];
  //   this->plane_coeff[1] = best_coefficients->values[1];
  //   this->plane_coeff[2] = best_coefficients->values[2];
  //   this->plane_coeff[3] = best_coefficients->values[3];
  //   // std::cout << "coefficients update" << std::endl;
  // }
  // else {
  //   best_coefficients->values[0] = this->plane_coeff[0];
  //   best_coefficients->values[1] = this->plane_coeff[1];
  //   best_coefficients->values[2] = this->plane_coeff[2];
  //   best_coefficients->values[3] = this->plane_coeff[3];
  //   std::cout << "coefficients reserved" << std::endl;
  // }

  // // Extract the plane from the point cloud
  // // Configure the point cloud extraction algorithm
  // extract_.setInputCloud(downsampled);
  // extract_.setIndices(best_inliers);
  // extract_.setNegative(false);
  // // Execute the point cloud extraction algorithm
  // extract_.filter(*segmented);

  // // Publish the segmented point cloud
  // sensor_msgs::PointCloud2 output;
  // pcl::toROSMsg(*segmented, output);
  // output.header.frame_id = "camera_link";
  // this->pub_.publish(output);

  // /* ***********************************************
  // *********** Multiple Plane Segment End************
  // *********************************************** */

}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::extractPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  // auto start = std::chrono::high_resolution_clock::now(); 
  for(int i = 0;i < cloud->points.size();i++){
    pcl::PointXYZ pt = cloud->points[i];
    float distance = coefficients->values[0] * pt.x + coefficients->values[1] * pt.y + coefficients->values[2] * pt.z + coefficients->values[3];
    if (distance <= this->distance_threshold_) {
      inliers->indices.push_back(i);
    }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr calculatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*calculatedCloud);
  
  // auto end = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double, std::milli> elapsed = end - start;
  // std::cout << "Running Time " << elapsed.count() << " ms" << std::endl;
    
  return calculatedCloud;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_segmentation");
    ros::NodeHandle nh;
    PointCloudProcessor pcp(nh);
    ros::spin();
    return 0;
}
