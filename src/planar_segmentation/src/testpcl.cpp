#include <iostream>
#include <string>
#include <vector>
#include <cmath> // Required for M_PI

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>                 // For KdTree search method in clustering
#include <pcl/segmentation/extract_clusters.h> // For Euclidean Clustering
#include <pcl/common/centroid.h>               // For calculating centroids
#include <pcl/kdtree/kdtree_flann.h>           // For KdTreeFLANN used for searching centroids

// Define PointT type alias for convenience
using PointT = pcl::PointXYZRGB;

int main()
{
  // --- Point Cloud Setup ---
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>); // Cloud containing only plane points

  pcl::PCDReader cloud_reader;
  pcl::PCDWriter cloud_writer;
  std::string path = "/home/7_fri/Downloads/"; // Make sure this path is correct

  // --- Load Cloud ---
  if (cloud_reader.read(path + "cloud.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file cloud.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << cloud->width * cloud->height << " data points from cloud.pcd." << std::endl;

  // --- Optional Transformation ---
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  float angle = M_PI; // 180 degrees
  // Applying rotation around Z then Y (check if this order gives desired result)
  transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
  transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
  pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
  std::cout << "Applied transformation." << std::endl;

  // --- Optional PassThrough Filter ---
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transformed_cloud); // Filter the transformed cloud
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.5, -0.2); // Adjust limits as needed
  pass.filter(*filtered_cloud);
  std::cout << "Applied PassThrough filter. Points remaining: " << filtered_cloud->size() << std::endl;

  if (filtered_cloud->empty())
  {
    PCL_ERROR("Cloud is empty after filtering. Cannot perform segmentation.\n");
    return (-1);
  }

  // --- Planar Segmentation (RANSAC) ---
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointT> plane_seg;

  plane_seg.setOptimizeCoefficients(true); // Optional: Refine coefficients
  plane_seg.setModelType(pcl::SACMODEL_PLANE);
  plane_seg.setMethodType(pcl::SAC_RANSAC);
  plane_seg.setDistanceThreshold(0.01); // 1cm tolerance - Adjust as needed
  plane_seg.setMaxIterations(100);      // Max iterations for RANSAC

  plane_seg.setInputCloud(filtered_cloud);
  plane_seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty())
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    return (-1);
  }

  std::cerr << "Plane coefficients: " << *coefficients << std::endl;
  std::cout << "Found " << inliers->indices.size() << " inliers for the plane model." << std::endl;

  // --- Extract Planar Points ---
  pcl::ExtractIndices<PointT> extract_indices;
  extract_indices.setInputCloud(filtered_cloud);
  extract_indices.setIndices(inliers);
  extract_indices.setNegative(false);   // Keep the points specified by the indices
  extract_indices.filter(*plane_cloud); // Store planar points in plane_cloud
  std::cout << "Extracted planar points into plane_cloud." << std::endl;

  // --- Write the segmented plane cloud (optional) ---
  cloud_writer.write<PointT>(path + "plane_seg_cloud.pcd", *plane_cloud, false);
  std::cout << "Saved planar points to plane_seg_cloud.pcd" << std::endl;

  // --- Euclidean Clustering on the Planar Points ---
  std::cout << "Starting clustering on the planar points..." << std::endl;

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(plane_cloud); // Use the extracted plane points

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;

  // Define clustering parameters (TUNE THESE!)
  double cluster_tolerance = 0.05; // 5cm - Max distance between points in cluster
  int min_cluster_size = 50;       // Minimum points per cluster
  int max_cluster_size = 10000;    // Maximum points per cluster

  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(plane_cloud); // Cluster the points belonging to the plane
  ec.extract(cluster_indices);

  if (cluster_indices.empty())
  {
    PCL_WARN("No clusters found on the planar surface after extraction.\n");
    // Decide if this is an error or just means no distinct groups
    // return (-1); // Optional: exit if no clusters are found
  }
  else
  {
    PCL_INFO("Found %ld clusters on the planar surface.\n", cluster_indices.size());
  }
  std::vector<pcl::PointIndices> filtered_clusters_by_points;
  for (const auto &cluster : cluster_indices)
  {
    if (cluster.indices.size() >= 250)
    {
      filtered_clusters_by_points.push_back(cluster);
    }
  }

  pcl::PointIndices::Ptr combined_filtered_indices(new pcl::PointIndices);
  for (const auto &filtered_cluster : filtered_clusters_by_points)
  { // Use the filtered list
    combined_filtered_indices->indices.insert(
        combined_filtered_indices->indices.end(),
        filtered_cluster.indices.begin(),
        filtered_cluster.indices.end());
  }

  // 2. Extract points using the combined indices
  pcl::PointCloud<PointT>::Ptr filtered_clusters_cloud(new pcl::PointCloud<PointT>);
  if (!combined_filtered_indices->indices.empty())
  {
    // Re-use the extract_indices object, but set new input cloud and indices
    extract_indices.setInputCloud(plane_cloud);            // Extract FROM the cloud containing all plane points
    extract_indices.setIndices(combined_filtered_indices); // Use the combined indices of FILTERED clusters
    extract_indices.setNegative(false);                    // Keep these points
    extract_indices.filter(*filtered_clusters_cloud);      // Output cloud with only filtered cluster points

    std::cout << "Created cloud with points from filtered clusters (" << filtered_clusters_cloud->size() << " points)." << std::endl;

    // 3. Write the final cloud containing only filtered cluster points
    cloud_writer.write<PointT>(path + "non_plane_seg.pcd", *filtered_clusters_cloud, false);

    // --- Calculate Centroids and Find Closest ---
    if (!cluster_indices.empty())
    {
      pcl::PointCloud<PointT>::Ptr centroid_cloud(new pcl::PointCloud<PointT>);
      // std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> centroid_vectors; // If you need raw vectors

      std::cout << "Calculating centroids..." << std::endl;
      for (const auto &indices : cluster_indices)
      {
        // Create a cloud for the current cluster
        pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
        // *** IMPORTANT: Use plane_cloud as the source for copying ***
        pcl::copyPointCloud(*plane_cloud, indices.indices, *cluster_cloud);

        // Calculate the centroid
        Eigen::Vector4f centroid_vec;
        pcl::compute3DCentroid(*cluster_cloud, centroid_vec);

        // Store the centroid point
        PointT centroid_point;
        centroid_point.x = centroid_vec[0];
        centroid_point.y = centroid_vec[1];
        centroid_point.z = centroid_vec[2];
        // Assign default color or average cluster color if desired
        centroid_point.r = 255;
        centroid_point.g = 0;
        centroid_point.b = 0; // Example: Red centroids
        centroid_cloud->push_back(centroid_point);
        // centroid_vectors.push_back(centroid_vec);
      }

      if (centroid_cloud->empty())
      {
        PCL_WARN("Centroid cloud is empty even though clusters were found. Check logic.\n");
        return (-1); // Indicate error
      }

      PCL_INFO("Calculated %ld centroids.\n", centroid_cloud->size());

      // --- Build KdTree from Centroids ---
      pcl::KdTreeFLANN<PointT>::Ptr kdtree_centroids(new pcl::KdTreeFLANN<PointT>);
      kdtree_centroids->setInputCloud(centroid_cloud);

      // --- Define Reference Point (e.g., origin) ---
      PointT reference_point;
      reference_point.x = 0.0;
      reference_point.y = 0.0;
      reference_point.z = 0.0;
      // reference_point.r = 0; reference_point.g = 255; reference_point.b = 0; // Color doesn't matter for search

      // --- Query KdTree for the closest centroid ---
      std::vector<int> pointIdxNKNSearch(1);         // To store index of the nearest neighbor
      std::vector<float> pointNKNSquaredDistance(1); // To store the squared distance

      std::cout << "Searching for centroid closest to ("
                << reference_point.x << ", " << reference_point.y << ", " << reference_point.z << ")" << std::endl;

      if (kdtree_centroids->nearestKSearch(reference_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        int closest_centroid_index = pointIdxNKNSearch[0];
        // Declare the variable to hold the result
        PointT closest_centroid = (*centroid_cloud)[closest_centroid_index];

        PCL_INFO("Closest cluster centroid found at index %d: (%f, %f, %f) with squared distance %f\n",
                 closest_centroid_index,
                 closest_centroid.x, closest_centroid.y, closest_centroid.z,
                 pointNKNSquaredDistance[0]);

        // You can now use the 'closest_centroid' variable
        // Example: Write the centroid cloud to visualize
        cloud_writer.write<PointT>(path + "centroids.pcd", *centroid_cloud, false);
        std::cout << "Saved centroids to centroids.pcd" << std::endl;
      }
      else
      {
        PCL_WARN("KdTree search for nearest centroid failed.\n");
        // Handle case where search fails (e.g., centroid cloud was empty, though checked earlier)
      }
    } // End if (!cluster_indices.empty())

    std::cout << "Processing finished successfully." << std::endl;
    return (0); // Indicate successful execution
  }
}