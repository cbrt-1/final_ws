#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <rclcpp/qos.hpp>

#include <pcl_conversions/pcl_conversions.h> // For converting ROS messages to PCL types
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_map", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  
        // --- Add Publisher Initialization ---
        // Define the output topic name
        std::string output_topic = "planar_segmentation/output_points"; // Or get from parameter
        // Use the same QoS or define another one
        auto qos_profile = rclcpp::QoS(10); // Or rclcpp::SensorDataQoS()
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos_profile);
        RCLCPP_INFO(this->get_logger(), "Publishing non-planar points on: '%s'", publisher_->get_topic_name());
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 & msg) const
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      try
      {
        pcl::fromROSMsg(msg, *cloud);
        if (cloud->empty())
        {
          RCLCPP_WARN(this->get_logger(), "Received empty point cloud message.");
          return;
        }
        RCLCPP_DEBUG(this->get_logger(), "Successfully converted ROS message to PCL cloud with %zu points.", cloud->size());
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert ROS PointCloud2 to PCL: %s", e.what());
        return;
      }
      catch (...)
      {
        RCLCPP_ERROR(this->get_logger(), "An unknown error occurred during PCL conversion.");
        return;
      }
      

      pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
      plane_seg.setModelType(pcl::SACMODEL_PLANE);
      plane_seg.setMethodType(pcl::SAC_RANSAC);
      plane_seg.setDistanceThreshold(100);
      plane_seg.setInputCloud(cloud);
      plane_seg.segment(*inliers,*coefficients);
  
  
      pcl::ExtractIndices<pcl::PointXYZ> extract_indicies;
      extract_indicies.setInputCloud(cloud);
      extract_indicies.setIndices(inliers);
      extract_indicies.setNegative(false);
      extract_indicies.filter(*plane_seg_cloud);

      // --- Add Conversion and Publishing ---
      if (inliers->indices.empty()) // Check if a plane was found
      {
           RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset. Publishing original cloud.");
           // Publish the original cloud if no plane is found (optional behavior)
           publisher_->publish(msg);
      }
      else if (!plane_seg_cloud->empty()) // Check if there are points left after removing the plane
      {
          sensor_msgs::msg::PointCloud2 output_msg;
          // Convert the PCL cloud (without the plane) back to a ROS message
          pcl::toROSMsg(*plane_seg_cloud, output_msg);

          // IMPORTANT: Set the header of the output message to match the input message
          // This preserves the frame_id and timestamp for TF and visualization
          output_msg.header = msg.header;

          // Publish the message
          publisher_->publish(output_msg);
          RCLCPP_DEBUG(this->get_logger(), "Published point cloud with plane removed (%zu points).", plane_seg_cloud->size());
      }
      else
      {
          RCLCPP_INFO(this->get_logger(), "Plane removal resulted in an empty cloud. Nothing published.");
          // Optional: Publish an empty message if needed by downstream nodes
          // sensor_msgs::msg::PointCloud2 empty_msg;
          // empty_msg.header = msg.header;
          // publisher_->publish(empty_msg);
      }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}