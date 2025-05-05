#include <memory>
#include <string>
#include <vector>
#include <cmath> // Required for M_PI
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <builtin_interfaces/msg/time.hpp> // Include for getting time

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "planar_segmentation/srv/set_reference_point.hpp"

// Use PointXYZRGB based on your PCL code using .r, .g, .b later
using PointT = pcl::PointXYZRGB;
using SetReferencePoint = planar_segmentation::srv::SetReferencePoint;
using std::placeholders::_1; // For std::bind


class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        // --- Subscriber Initialization ---
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud_map", // Input topic
            10,          // QoS depth or profile
            std::bind(&MinimalSubscriber::topic_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscription_->get_topic_name());

        // --- Publisher Initialization ---
        std::string output_topic = "planar_segmentation/closest_centroid"; // Changed topic name for clarity
        auto qos_profile = rclcpp::QoS(10);
        // Initialize the member variable publisher_
        centroid_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(output_topic, qos_profile);
        RCLCPP_INFO(this->get_logger(), "Publishing closest centroid on: '%s'", centroid_publisher_->get_topic_name());
    }

private:
    // *** MEMBER VARIABLE DECLARATIONS MOVED HERE ***
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_publisher_; // Renamed for clarity

    // *** TOPIC CALLBACK ***
    // Changed signature to use SharedPtr to easily access header info
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Check for null message pointer
        if (!msg)
        {
            RCLCPP_ERROR(this->get_logger(), "Received null PointCloud2 message pointer");
            return;
        }
      

        // --- PCL Cloud Conversion ---
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        try
        {
            pcl::fromROSMsg(*msg, *cloud); // Use *msg because it's now a SharedPtr
            if (cloud->empty())
            {
                RCLCPP_WARN(this->get_logger(), "Converted PCL cloud is empty!");
                // Optionally publish a default "not found" point here if needed
                // publishNotFoundPoint(msg->header); // Example helper function
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Successfully converted cloud. Size: %zu points", cloud->size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert PointCloud2 to PCL: %s", e.what());
            return;
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown exception during PCL conversion.");
            return;
        }

        // --- PCL Processing Pipeline ---
        pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>);

        // --- Transformation (Optional) ---
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        // Example: Rotate if needed - make sure M_PI is defined (needs <cmath>)
        // float angle = M_PI;
        // transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
        // transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
        // pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
        transformed_cloud = cloud; // Use original cloud if no transform applied
        // RCLCPP_INFO(this->get_logger(), "Applied transformation (if any).");

        // --- PassThrough Filter (Optional) ---
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(transformed_cloud);
        pass.setFilterFieldName("y"); // Example filter field
        pass.setFilterLimits(-0.5, 0.5); // Example filter limits
        pass.filter(*filtered_cloud);
        RCLCPP_INFO(this->get_logger(), "Applied PassThrough filter. Points remaining: %zu", filtered_cloud->size());


        if (filtered_cloud->empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Cloud is empty after filtering. Cannot perform segmentation.");
            publishNotFoundPoint(msg->header); // Publish default point
            return; // Exit callback
        }

        // --- Planar Segmentation (RANSAC) ---
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointT> plane_seg;

        plane_seg.setOptimizeCoefficients(true);
        plane_seg.setModelType(pcl::SACMODEL_PLANE);
        plane_seg.setMethodType(pcl::SAC_RANSAC);
        plane_seg.setDistanceThreshold(0.01); // Adjust as needed
        plane_seg.setMaxIterations(100);
        plane_seg.setInputCloud(filtered_cloud);
        plane_seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not estimate a planar model for the given dataset.");
            publishNotFoundPoint(msg->header); // Publish default point
            return; // Exit callback
        }

        RCLCPP_INFO(this->get_logger(), "Found %zu inliers for the plane model.", inliers->indices.size());
        // std::cerr << "Plane coefficients: " << *coefficients << std::endl; // Optional print

        // --- Extract Planar Points ---
        pcl::ExtractIndices<PointT> extract_indices;
        extract_indices.setInputCloud(filtered_cloud);
        extract_indices.setIndices(inliers);
        extract_indices.setNegative(false); // Keep the planar points
        extract_indices.filter(*plane_cloud);
        RCLCPP_INFO(this->get_logger(), "Extracted %zu planar points into plane_cloud.", plane_cloud->size());

        if (plane_cloud->empty()) {
            RCLCPP_ERROR(this->get_logger(), "Plane cloud is empty after extraction, cannot cluster.");
            publishNotFoundPoint(msg->header);
            return;
        }

        // --- Euclidean Clustering on the Planar Points ---
        RCLCPP_INFO(this->get_logger(), "Starting clustering on the planar points...");
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(plane_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        double cluster_tolerance = 0.05; // Tune these parameters
        int min_cluster_size = 50;
        int max_cluster_size = 10000;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(plane_cloud);
        ec.extract(cluster_indices);

        if (cluster_indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No clusters found on the planar surface after extraction.");
            publishNotFoundPoint(msg->header); // Publish default point
            return; // Exit callback
        }

        RCLCPP_INFO(this->get_logger(), "Found %ld clusters on the planar surface.", cluster_indices.size());

        // --- Filter clusters by size (Optional but potentially useful) ---
        std::vector<pcl::PointIndices> filtered_clusters_by_points;
        int min_points_for_filtered_cluster = 100; // Example threshold
        for (const auto &cluster : cluster_indices)
        {
            if (cluster.indices.size() >= min_points_for_filtered_cluster)
            {
                filtered_clusters_by_points.push_back(cluster);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Found %ld clusters meeting size criteria >= %d points.",
                    filtered_clusters_by_points.size(), min_points_for_filtered_cluster);

        if (filtered_clusters_by_points.empty()) {
             RCLCPP_WARN(this->get_logger(), "No clusters left after size filtering.");
             publishNotFoundPoint(msg->header);
             return;
        }

        // --- Calculate Centroids of Filtered Clusters ---
        pcl::PointCloud<PointT>::Ptr centroid_cloud(new pcl::PointCloud<PointT>);
        RCLCPP_INFO(this->get_logger(), "Calculating centroids for filtered clusters...");

        // Use the filtered list here
        for (const auto &indices : filtered_clusters_by_points)
        {
            Eigen::Vector4f centroid_vec;
            // Compute centroid directly using indices from the *original plane cloud*
            pcl::compute3DCentroid(*plane_cloud, indices.indices, centroid_vec);

            PointT centroid_point;
            centroid_point.x = centroid_vec[0];
            centroid_point.y = centroid_vec[1];
            centroid_point.z = centroid_vec[2];
            centroid_point.r = 255; centroid_point.g = 0; centroid_point.b = 0; // Red centroids
            centroid_cloud->push_back(centroid_point);
        }

        if (centroid_cloud->empty())
        {
            // This shouldn't happen if filtered_clusters_by_points wasn't empty, but check anyway
            RCLCPP_WARN(this->get_logger(), "Centroid cloud is unexpectedly empty.");
            publishNotFoundPoint(msg->header);
            return;
        }
         RCLCPP_INFO(this->get_logger(), "Calculated %zu centroids.", centroid_cloud->size());


        // --- Build KdTree from Centroids ---
        pcl::KdTreeFLANN<PointT>::Ptr kdtree_centroids(new pcl::KdTreeFLANN<PointT>);
        kdtree_centroids->setInputCloud(centroid_cloud);

        // --- Define Reference Point (e.g., origin) ---
        PointT reference_point;
        reference_point.x = 0.0; reference_point.y = 0.0; reference_point.z = 0.0;

        // --- Query KdTree for the closest centroid ---
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        RCLCPP_INFO(this->get_logger(), "Searching for centroid closest to origin...");

        if (kdtree_centroids->nearestKSearch(reference_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            int closest_centroid_index = pointIdxNKNSearch[0];
            if (closest_centroid_index >= centroid_cloud->points.size()) {
                 RCLCPP_ERROR(this->get_logger(), "KdTree returned invalid index %d for centroid cloud size %zu",
                              closest_centroid_index, centroid_cloud->points.size());
                 publishNotFoundPoint(msg->header);
                 return;
            }

            PointT closest_centroid = (*centroid_cloud)[closest_centroid_index];

            RCLCPP_INFO(this->get_logger(), "Closest cluster centroid found at index %d: (%f, %f, %f) dist^2: %f",
                       closest_centroid_index,
                       closest_centroid.x, closest_centroid.y, closest_centroid.z,
                       pointNKNSquaredDistance[0]);

            // --- Create and Publish PointStamped ---
            auto point_stamped_msg = geometry_msgs::msg::PointStamped();
            point_stamped_msg.header.stamp = msg->header.stamp;       // Use stamp from input message
            point_stamped_msg.header.frame_id = msg->header.frame_id; // Use frame from input message
            point_stamped_msg.point.x = static_cast<double>(closest_centroid.x);
            point_stamped_msg.point.y = static_cast<double>(closest_centroid.y);
            point_stamped_msg.point.z = static_cast<double>(closest_centroid.z);

            centroid_publisher_->publish(point_stamped_msg); // Publish using the member publisher
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "KdTree search for nearest centroid failed (kdtree empty or other issue).");
            publishNotFoundPoint(msg->header); // Publish default point
        }

        RCLCPP_INFO(this->get_logger(), "Processing finished for this message.");

    } // <-- End of topic_callback function

    // Helper function to publish a default "not found" point
    void publishNotFoundPoint(const std_msgs::msg::Header& header)
    {
        auto point_stamped_msg = geometry_msgs::msg::PointStamped();
        point_stamped_msg.header.stamp = header.stamp;       // Use original stamp
        point_stamped_msg.header.frame_id = header.frame_id; // Use original frame
        point_stamped_msg.point.x = -1.0; // Use sentinel values
        point_stamped_msg.point.y = -1.0;
        point_stamped_msg.point.z = -1.0;
        centroid_publisher_->publish(point_stamped_msg);
        RCLCPP_WARN(this->get_logger(), "Published 'not found' point (-1,-1,-1)");
    }


}; // <-- *** END OF MinimalSubscriber CLASS DEFINITION ***

// *** MAIN FUNCTION MOVED OUTSIDE THE CLASS ***
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing planar segmentation node...");
    auto node = std::make_shared<MinimalSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Node shutting down.");
    return 0;
}