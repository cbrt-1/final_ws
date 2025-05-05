#include <memory>
#include <string>
#include <vector>
#include <cmath>    // Required for M_PI (if used) and potentially std::numeric_limits
#include <iostream>
#include <limits>   // *** ADDED for std::numeric_limits (for NaN) ***
#include <mutex>    // *** ADDED for std::mutex ***

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/point_stamped.hpp> // Still needed for publisher
#include <rclcpp/qos.hpp>
// #include <builtin_interfaces/msg/time.hpp> // Not strictly needed if just using header
#include <std_msgs/msg/header.hpp>         // *** ADDED for publishNotFoundPoint header type ***

// *** ADDED Service Include ***
#include "planar_segmentation/srv/set_reference_point.hpp" // Generated from your .srv file

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> // Keep if you use it elsewhere, else remove
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

// Use PointXYZRGB based on your PCL code using .r, .g, .b later
using PointT = pcl::PointXYZRGB;
// *** ADDED Service Type Alias ***
using SetReferencePoint = planar_segmentation::srv::SetReferencePoint;
using std::placeholders::_1;
using std::placeholders::_2; // *** ADDED for service bind ***


class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber") // Consider renaming node e.g., "planar_segmentation_node"
    {
        // --- Initialize Reference Point to Origin (Thread-Safe) ---
        { // Scope for lock guard during initialization
            std::lock_guard<std::mutex> lock(reference_point_mutex_);
            // Initialize the member variable holding the reference point
            reference_point_.x = 0.0f;
            reference_point_.y = 0.0f;
            reference_point_.z = 0.0f;
            RCLCPP_INFO(this->get_logger(), "Default reference point initialized to (0,0,0).");
        }

        // --- Subscriber Initialization ---
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud_map", // Input topic
             rclcpp::SensorDataQoS(), // *** Use SensorDataQoS for point clouds ***
            std::bind(&MinimalSubscriber::topic_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscription_->get_topic_name());

        // --- Publisher Initialization ---
        std::string output_topic = "planar_segmentation/closest_centroid"; // Changed topic name for clarity
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // *** Use KeepLast or other appropriate QoS ***
        centroid_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(output_topic, qos_profile);
        RCLCPP_INFO(this->get_logger(), "Publishing closest centroid on: '%s'", centroid_publisher_->get_topic_name());

        // --- Service Server Initialization --- // *** ADDED ***
        std::string service_name = "set_reference_point"; // Name of the service
        set_point_service_ = this->create_service<SetReferencePoint>(
            service_name,
            std::bind(&MinimalSubscriber::handle_set_reference_point, this, _1, _2)); // Bind to handler
        RCLCPP_INFO(this->get_logger(), "Service '%s' created.", service_name.c_str());

    } // End Constructor

private:
    // --- Member Variables ---
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_publisher_; // Renamed for clarity

    // *** ADDED Service Server Member ***
    rclcpp::Service<SetReferencePoint>::SharedPtr set_point_service_;

    // *** ADDED Shared State for Reference Point ***
    PointT reference_point_;          // The reference point (updated by service)
    std::mutex reference_point_mutex_; // Mutex to protect access


    // --- Service Callback (Handles float64 x, y, z request) --- // *** ADDED ***
    /**
     * @brief Service handler to update the reference point using x, y, z coordinates.
     * Assumes input coordinates are in the correct frame.
     */
    void handle_set_reference_point(
        const std::shared_ptr<SetReferencePoint::Request> request,
        std::shared_ptr<SetReferencePoint::Response> response)
    {
        (void)response; // Mark response as unused since it's empty

        const std::string& service_name = set_point_service_->get_service_name(); // Get name for logging
        RCLCPP_INFO(this->get_logger(), "Service '%s': Received request to set reference point.", service_name);
        RCLCPP_WARN(this->get_logger(), "Service '%s': Assuming input coordinates (x,y,z) are in the correct processing frame.", service_name);

        // Update the member variable, protected by the mutex
        { // Scope for the lock guard
            std::lock_guard<std::mutex> lock(reference_point_mutex_);
            // Access fields directly from the request (float64 x, y, z)
            reference_point_.x = static_cast<float>(request->x); // Cast double to float for PointT
            reference_point_.y = static_cast<float>(request->y);
            reference_point_.z = static_cast<float>(request->z);
            // Optional: Update RGB if needed for visualization, e.g., make it visually distinct
            // reference_point_.r = 0; reference_point_.g = 0; reference_point_.b = 255; // Make it blue?
        }

        RCLCPP_INFO(this->get_logger(), "Service '%s': Reference point updated to (%.3f, %.3f, %.3f).",
                service_name,
                reference_point_.x, reference_point_.y, reference_point_.z);
    } // End handle_set_reference_point


    // --- Topic Callback ---
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
                publishNotFoundPoint(msg->header); // Call helper
                return;
            }
            // Limit excessive logging - use DEBUG or log less often
            // RCLCPP_INFO(this->get_logger(), "Successfully converted cloud. Size: %zu points", cloud->size());
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
        // ... (Keep your transform logic or assign cloud directly) ...
        transformed_cloud = cloud;

        // --- PassThrough Filter (Optional) ---
        // ... (Keep your PassThrough logic with hardcoded values) ...
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(transformed_cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-0.5, 0.5);
        pass.filter(*filtered_cloud);
        // RCLCPP_INFO(this->get_logger(), "Applied PassThrough filter. Points remaining: %zu", filtered_cloud->size()); // Maybe DEBUG level

        if (filtered_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Cloud is empty after filtering. Cannot perform segmentation."); // Use WARN
            publishNotFoundPoint(msg->header); // Publish default point
            return; // Exit callback
        }

        // --- Planar Segmentation (RANSAC) ---
        // ... (Keep your Segmentation logic with hardcoded values) ...
         pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointT> plane_seg;
        plane_seg.setOptimizeCoefficients(true);
        plane_seg.setModelType(pcl::SACMODEL_PLANE);
        plane_seg.setMethodType(pcl::SAC_RANSAC);
        plane_seg.setDistanceThreshold(0.01);
        plane_seg.setMaxIterations(100);
        plane_seg.setInputCloud(filtered_cloud);
        plane_seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset."); // Use WARN
            publishNotFoundPoint(msg->header); // Publish default point
            return; // Exit callback
        }
       // RCLCPP_INFO(this->get_logger(), "Found %zu inliers for the plane model.", inliers->indices.size()); // DEBUG?

        // --- Extract Planar Points ---
        // ... (Keep your Extraction logic) ...
        pcl::ExtractIndices<PointT> extract_indices;
        extract_indices.setInputCloud(filtered_cloud);
        extract_indices.setIndices(inliers);
        extract_indices.setNegative(false); // Keep the planar points
        extract_indices.filter(*plane_cloud);
       // RCLCPP_INFO(this->get_logger(), "Extracted %zu planar points into plane_cloud.", plane_cloud->size()); // DEBUG?

        if (plane_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Plane cloud is empty after extraction, cannot cluster."); // Use WARN
            publishNotFoundPoint(msg->header);
            return;
        }

        // --- Euclidean Clustering on the Planar Points ---
       // RCLCPP_INFO(this->get_logger(), "Starting clustering on the planar points..."); // DEBUG?
        // ... (Keep your Clustering logic with hardcoded values) ...
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(plane_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.05);
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(10000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(plane_cloud);
        ec.extract(cluster_indices);

        if (cluster_indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No clusters found on the planar surface after extraction.");
            publishNotFoundPoint(msg->header); // Publish default point
            return; // Exit callback
        }
       // RCLCPP_INFO(this->get_logger(), "Found %ld clusters on the planar surface.", cluster_indices.size()); // DEBUG?

        // --- Filter clusters by size (Optional but potentially useful) ---
        // ... (Keep your Filtering logic with hardcoded values) ...
        std::vector<pcl::PointIndices> filtered_clusters_by_points;
        int min_points_for_filtered_cluster = 100;
        for (const auto &cluster : cluster_indices) {
            if (cluster.indices.size() >= static_cast<size_t>(min_points_for_filtered_cluster)) { // Safe cast
                filtered_clusters_by_points.push_back(cluster);
            }
        }
       // RCLCPP_INFO(this->get_logger(), "Found %ld clusters meeting size criteria >= %d points.",
       //             filtered_clusters_by_points.size(), min_points_for_filtered_cluster); // DEBUG?

        if (filtered_clusters_by_points.empty()) {
             RCLCPP_WARN(this->get_logger(), "No clusters left after size filtering.");
             publishNotFoundPoint(msg->header);
             return;
        }

        // --- Calculate Centroids of Filtered Clusters ---
        pcl::PointCloud<PointT>::Ptr centroid_cloud(new pcl::PointCloud<PointT>);
       // RCLCPP_INFO(this->get_logger(), "Calculating centroids for filtered clusters..."); // DEBUG?
       // ... (Keep centroid calculation logic) ...
        for (const auto &indices : filtered_clusters_by_points) {
            Eigen::Vector4f centroid_vec;
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
            RCLCPP_WARN(this->get_logger(), "Centroid cloud is unexpectedly empty.");
            publishNotFoundPoint(msg->header);
            return;
        }
       // RCLCPP_INFO(this->get_logger(), "Calculated %zu centroids.", centroid_cloud->size()); // DEBUG?

        // --- Build KdTree from Centroids ---
        pcl::KdTreeFLANN<PointT>::Ptr kdtree_centroids(new pcl::KdTreeFLANN<PointT>);
        kdtree_centroids->setInputCloud(centroid_cloud);

        // --- Get Current Reference Point (Thread-Safe) --- // *** MODIFIED SECTION ***
        // PointT reference_point; // --- REMOVE local definition ---
        // reference_point.x = 0.0; reference_point.y = 0.0; reference_point.z = 0.0; // --- REMOVE ---

        // *** ADD Thread-Safe Read ***
        PointT local_reference_point; // Local copy for this callback execution
        { // Scope for the lock guard
            std::lock_guard<std::mutex> lock(reference_point_mutex_);
            local_reference_point = reference_point_; // Copy the shared data
        }


        // --- Query KdTree for the closest centroid ---
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        // Update log message
        RCLCPP_DEBUG(this->get_logger(), "Searching for centroid closest to reference point (%.3f, %.3f, %.3f)...", // Use DEBUG
                    local_reference_point.x, local_reference_point.y, local_reference_point.z);

        // *** Use the thread-safe local copy in the search ***
        if (kdtree_centroids->nearestKSearch(local_reference_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            int closest_centroid_index = pointIdxNKNSearch[0];
            // Safer check using static_cast
            if (closest_centroid_index < 0 || static_cast<size_t>(closest_centroid_index) >= centroid_cloud->points.size()) {
                 RCLCPP_ERROR(this->get_logger(), "KdTree returned invalid index %d for centroid cloud size %zu",
                              closest_centroid_index, centroid_cloud->points.size());
                 publishNotFoundPoint(msg->header);
                 return;
            }

            PointT closest_centroid = (*centroid_cloud)[closest_centroid_index];

            // Log at INFO level as this is the primary output result
            RCLCPP_INFO(this->get_logger(), "Closest cluster centroid to ref pt found at index %d: (%.3f, %.3f, %.3f) dist^2: %.4f",
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

        // RCLCPP_DEBUG(this->get_logger(), "Processing finished for this message."); // Use DEBUG

    } // <-- End of topic_callback function

    // Helper function to publish a default "not found" point
    void publishNotFoundPoint(const std_msgs::msg::Header& header) // *** Takes const ref ***
    {
        auto point_stamped_msg = geometry_msgs::msg::PointStamped();
        point_stamped_msg.header.stamp = header.stamp;       // Use original stamp
        point_stamped_msg.header.frame_id = header.frame_id; // Use original frame
        // *** Use NaN as sentinel value ***
        const double nan_val = std::numeric_limits<double>::quiet_NaN();
        point_stamped_msg.point.x = nan_val;
        point_stamped_msg.point.y = nan_val;
        point_stamped_msg.point.z = nan_val;
        centroid_publisher_->publish(point_stamped_msg);
        RCLCPP_WARN(this->get_logger(), "Published 'not found' point (NaN, NaN, NaN)");
    } // End publishNotFoundPoint


}; // <-- *** END OF MinimalSubscriber CLASS DEFINITION ***

// *** MAIN FUNCTION - MODIFIED FOR MULTI-THREADING ***
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing node..."); // Generic message
    auto node = std::make_shared<MinimalSubscriber>();

    // *** Use MultiThreadedExecutor for service responsiveness ***
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Spinning node '%s' with MultiThreadedExecutor...", node->get_name()); // Use actual node name
    executor.spin(); // Allows concurrent callbacks

    // rclcpp::spin(node); // --- Replace this single-threaded spin ---

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Node shutting down.");
    return 0;
}