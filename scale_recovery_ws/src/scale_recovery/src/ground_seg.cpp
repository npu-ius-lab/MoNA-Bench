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
  const std::string err_output_topic_ = "/sync/pcl_err";
};

PointCloudProcessor::PointCloudProcessor(ros::NodeHandle nh)
{
  // Create a subscriber for the input point cloud
  sub_ = nh.subscribe(input_topic_, 1, &PointCloudProcessor::cloudCallback, this);

  // Create a publisher for the downsampled and segmented point cloud
  pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(output_topic_, 1);

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

  if (abs(coefficients->values[1]) > 0.90) {
    this->plane_coeff[0] = coefficients->values[0];
    this->plane_coeff[1] = coefficients->values[1];
    this->plane_coeff[2] = coefficients->values[2];
    this->plane_coeff[3] = coefficients->values[3];
    
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
