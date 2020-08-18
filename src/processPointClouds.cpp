// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    typename pcl::PointCloud<PointT>::Ptr c_filtered (new pcl::PointCloud<PointT>);
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*c_filtered);
    
    typename pcl::PointCloud<PointT>::Ptr c_region (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roi(true);
    roi.setInputCloud(c_filtered);
    roi.setMax(maxPoint);
    roi.setMin(minPoint);
    roi.filter(*c_region);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return c_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_obsta (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
      
    for (int index : inliers->indices)
      cloud_plane->points.push_back(cloud->points[index]);
    
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
  
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_obsta);
  

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obsta, cloud_plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // TODO:: Fill in this function to find inliers for the cloud.
    // Create a Segmentation Obj
    pcl::SACSegmentation<PointT> seg;
      pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
  
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers -> indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;     
    }
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // (Begin) RANSAC
    
    std::unordered_set<int> inliersResult;

    double dist;

    for (int i = 0; i < maxIterations; i++){
      
      std::unordered_set<int> inliers;
            
      // Create a plan with 3 random points
      while (inliers.size() < 3)
        inliers.insert(rand()%(cloud->points.size()));
      
      float p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z;
      
      // Analyse the first three points
      // Point 1
      auto itr = inliers.begin();      
      p1x = cloud->points[*itr].x;
      p1y = cloud->points[*itr].y;
      p1z = cloud->points[*itr].z;
      // Point 2
      itr++;
      p2x = cloud->points[*itr].x;
      p2y = cloud->points[*itr].y;
      p2z = cloud->points[*itr].z;
      // Point 3
      itr++;
      p3x = cloud->points[*itr].x;
      p3y = cloud->points[*itr].y;
      p3z = cloud->points[*itr].z;
      
      // Iterate through the all the points
      for( int index = 0; index < cloud->points.size(); index++){
        
        if (inliers.count(index)>0)
          continue;
        
        PointT point = cloud->points[index];
        float px = point.x;
        float py = point.y;
        float pz = point.z;

        // Measure the distance
        float a, b, c, d;
        float cp[3];
        
        float v1[3];
        v1[0] = p2x - p1x;
        v1[1] = p2y - p1y;
        v1[2] = p2z - p1z;
      
        float v2[3];
        v2[0] = p3x - p1x;
        v2[1] = p3y - p1y;
        v2[2] = p3z - p1z;
        
        // i =  (y2-y1) * (z3-z1)-(z2-z1) * (y3-y1)
        cp[0] = v1[1] * v2[2] - v1[2] * v2[1];
        cp[1] = -(v1[0] * v2[2] - v1[2] * v2[0]);
        cp[2] = v1[0] * v2[1] - v1[1] * v2[0];

        a = cp[0];
        b = cp[1];
        c = cp[2];
        d = -1 * ( cp[0]*p1x + cp[1]*p1y + cp[2]*p1z );
      
        dist = fabs(a * px + b * py + c * pz + d) / sqrt(a * a + b * b + c * c);

        // Insert points within distanceTol
        if (dist < distanceThreshold)
          inliers.insert(index);     

      }

      // Replace inliersResult with the bigger one
      if (inliers.size() > inliersResult.size())
        inliersResult = inliers;
    
    }

    // (End) RANSAC
    
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr plane_cloud (new pcl::PointCloud<PointT>);

    for(int i = 0; i < (cloud->points.size()); i++){

        if(inliersResult.count(i))
            obstacle_cloud->points.push_back(cloud->points[i]);
        else
            plane_cloud->points.push_back(cloud->points[i]);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(plane_cloud, obstacle_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    
    // Create a vector to store cluster indices
    std::vector<pcl::PointIndices> cluster_indices;
    
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
  
    ec.extract (cluster_indices);
  
    for( pcl::PointIndices getIndices: cluster_indices){
      
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      
      for (int index : getIndices.indices)
        cloud_cluster->points.push_back(cloud->points[index]);
      
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      
      clusters.push_back(cloud_cluster);
    
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
      processed[idx] = true;
  
      cluster.push_back(idx);

      std::vector<int> near_points = tree->search(cloud->points[idx],distanceTol);

      for(int index : near_points){
        if (!processed[index]){
          clusterHelper(index, cloud, cluster, processed, tree, distanceTol); 
        }
      }
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float clusterTolerance, int minSize, int maxSize)
{

  // TODO: Fill out this function to return list of indices for each cluster
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  
    std::vector<bool> processed(cloud->points.size(), false);

    for(int index = 0; index < cloud->points.size(); index++){
      
      if (processed[index])
        continue;

      std::vector<int> cluster_idx;
      typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

      clusterHelper(index, cloud, cluster_idx, processed, tree, clusterTolerance);
    
      if (cluster_idx.size() >= minSize && cluster_idx.size() <= maxSize)
      {
          for (int i = 0; i < cluster_idx.size(); i++)
            cluster->points.push_back(cloud->points[cluster_idx[i]]);

          cluster->width = cluster->points.size();
          cluster->height = 1;

          clusters.push_back(cluster);
      }
      else 
      {
          for (int i = 1; i < cluster_idx.size(); i++)
          {
              processed[cluster_idx[i]] = false;
          }
      }
      
    }

  return clusters;

}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

