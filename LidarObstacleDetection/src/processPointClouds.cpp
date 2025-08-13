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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    std::cerr << "Point Cloud Size - " << cloud->points.size() << std::endl;
    // voxel grid point reduction
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(filterRes, filterRes, filterRes);
    grid.filter(*cloudFiltered);
    //std::cerr << "Voxel Grid Size - " << cloudFiltered->points.size() << std::endl;

    // region filtering
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    typename pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);
    //std::cerr << "Region Filter Size - " << cloudRegion->points.size() << std::endl;

    // extract roof points and remove
    std::vector<int> indices;
    typename pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);
    //std::cerr << "Roof Size - " << indices.size() << std::endl;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int point : indices) inliers->indices.push_back(point);

    typename pcl::ExtractIndices<PointT> extractor;
    extractor.setNegative(true);
    extractor.setIndices(inliers);
    extractor.setInputCloud(cloudRegion);
    extractor.filter(*cloudRegion);
    std::cerr << "Final Region Size - " << cloudRegion->points.size() << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlanes(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inlierResult;
    srand(time(NULL));

    int iter = 0;
    while (iter < maxIterations) {
        std::unordered_set<int> inlierSet;
        while (inlierSet.size() < 3) inlierSet.insert(rand() % cloud->points.size());

        auto it = inlierSet.begin();
        float x1{ cloud->points[*it].x }, y1{ cloud->points[*it].y }, z1{ cloud->points[*it].z };
        it++;
        float x2{ cloud->points[*it].x }, y2{ cloud->points[*it].y }, z2{ cloud->points[*it].z };
        it++;
        float x3{ cloud->points[*it].x }, y3{ cloud->points[*it].y }, z3{ cloud->points[*it].z };

        float a{ ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1)) };
        float b{ ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1)) };
        float c{ ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1)) };
        float d{ -(a * x1 + b * y1 + c * z1) };

        for (int idx = 0; idx < cloud->points.size(); idx++) {
            if (inlierSet.count(idx)) continue;

            float x{ cloud->points[idx].x }, y{ cloud->points[idx].y }, z{ cloud->points[idx].z };
            float dist{ fabs(a * x + b * y + c * z + d) / sqrt((a * a) + (b * b) + (c * c)) };

            if (dist <= distanceTol) inlierSet.insert(idx);
        }

        if (inlierSet.size() > inlierResult.size()) inlierResult = inlierSet;
        iter++;
    }

    return inlierResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr roadCloud(new pcl::PointCloud<PointT>());

    for (auto index : inliers->indices) {
        roadCloud->points.push_back(cloud->points[index]);
    }
    typename pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(cloud);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    extractor.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentCustom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliers = RansacPlanes(cloud, maxIterations, distanceThreshold);
    if (inliers.size() == 0) {
        std::cout << "Could not estimate the planar model for these points" << std::endl;
    }

    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr roadCloud(new pcl::PointCloud<PointT>());

    for (int idx = 0; idx < cloud->points.size(); idx++) {
        if (inliers.count(idx)) {
            roadCloud->points.push_back(cloud->points[idx]);
        }
        else {
            obstCloud->points.push_back(cloud->points[idx]);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, roadCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    typename pcl::SACSegmentation<PointT> segmentor;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setDistanceThreshold(distanceThreshold);
    segmentor.setMaxIterations(maxIterations);

    segmentor.setInputCloud(cloud);
    segmentor.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cout << "Could not estimate the planar model for these points" << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;

    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto& cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*cloud)[idx]);
        } 
        cloud_cluster->width = cloud_cluster->size();
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
void ProcessPointClouds<PointT>::Proximity(int index, const typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, typename KdTree<PointT>* tree, float distanceTol)
{
    processed[index] = true;

    cluster.push_back(index);
    std::vector<int> nearby = tree->search(cloud->points[index], distanceTol);
    for (int idx : nearby) {
        if (!processed[idx]) {
            Proximity(idx, cloud, cluster, processed, tree, distanceTol);
        }
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr cloud, typename KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(cloud->points.size(), false);
    for (int i = 0; i < cloud->points.size(); i++) {
        if (processed[i]) continue;

        std::vector<int> cluster;
        Proximity(i, cloud, cluster, processed, tree, distanceTol);

        if (cluster.size() >= minSize && cluster.size() <= maxSize)
            clusters.push_back(cluster);
    }

    return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringCustom(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename KdTree<PointT>* tree(new KdTree<PointT>);
    tree->insertCloud(cloud);

    std::vector<std::vector<int>> cluster_indices = euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);
    for (std::vector<int> cluster : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (int idx : cluster) {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));   /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                    ///    the signs are different and the box doesn't get correctly oriented in some cases.

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    typename PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    BoxQ box;
    box.bboxQuaternion = bboxQuaternion;
    box.bboxTransform = bboxTransform;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

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