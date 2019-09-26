// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


template<typename PointT>
void getProximitry(
        std::vector<PointT>& points,
        const int pIdx,
        KdTree<PointT>* tree,
        float distanceTol ,
        std::vector<bool> &processed ,
        std::vector<int>& cluster,
         int maxSize
        ){


        processed.at(pIdx) = true;
        cluster.push_back(pIdx);
        if (cluster.size() == maxSize)
        {
            std::cout << " cluster has max size, find next Cluster " << std::endl;

            return;
        }
        auto directNighbour = tree->search(points.at(pIdx) , distanceTol );
        for (int i = 0 ; i < directNighbour.size() ; i++ )
        {
            auto testIdx = directNighbour.at(i);
            if (processed.at(testIdx) == false)
            {
            getProximitry(points , testIdx , tree , distanceTol , processed ,  cluster  , maxSize);
            }
        }
}

template<typename PointT>
std::vector<std::vector<int>> euclideanCluster(std::vector<PointT>& points,
                                               KdTree<PointT>* tree,
                                               float distanceTol,
                                               int minSize,
                                               int maxSize)
{

    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size() , false);

    for (int i = 0 ; i < points.size() ; i++)
    {
        if (processed.at(i) == false)
        {
            std::vector<int> cluster;
            getProximitry(points , i , tree , distanceTol , processed ,  cluster , maxSize );
            if (minSize <=  cluster.size()){
            clusters.push_back(cluster);
            }
            else
            {
                std::cout << " cluster to small, drop it " << std::endl;

            }

        }

    }
    return clusters;

}







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
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::ransacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    pcl::PointIndices::Ptr result(new pcl::PointIndices);
    srand(time(NULL));
    const int size = cloud->points.size();



    if (size < 3 )
    {
    // not enoght points to create plane
      return result;
    }
    // For max iterations
    for(int i = 0 ; i < maxIterations ; i++ ){
        std::unordered_set<int> workSet;
        do{
            workSet.insert(std::rand()%size);

        }while(workSet.size() < 3 );

        auto planePointIt = workSet.begin();
        auto p1 = cloud->points.at(*planePointIt);
        planePointIt++;
        auto p2 = cloud->points.at(*planePointIt);
        planePointIt++;
        auto p3 = cloud->points.at(*planePointIt);

        auto a = (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y);
        auto b = (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z);
        auto c = (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x);
        auto d = -(a*p1.x+b*p1.y+c*p1.z);



        for (int j = 0 ; j < size ; j++)
        {
            auto tp = cloud->at(j);
            auto dist = abs(a*tp.x + b*tp.y + c*tp.z + d ) / sqrt(pow(a,2) + pow(b,2) + pow(c,2));
            if (dist <= distanceTol)
            {
                workSet.insert(j);
            }

        }
        if (workSet.size() > inliersResult.size())
        {
            inliersResult = workSet;
        }

    }


    result->indices.resize(inliersResult.size());
    int i= 0 ;
    for(auto idx : inliersResult )
    {
    result->indices[i]=(idx);
    i++;

    }

    return result;

}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_filteredVoxl(new pcl::PointCloud<PointT>);
     typename pcl::PointCloud<PointT>::Ptr cloud_filteredRegion(new pcl::PointCloud<PointT>);

    // first get downsampling with Voxels
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filteredVoxl);
    // Get region Filtering

    pcl::CropBox< PointT > crop(true);
    crop.setInputCloud(cloud_filteredVoxl);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.filter(*cloud_filteredRegion);


    std::vector<int> idx;
    crop.setInputCloud(cloud_filteredRegion);
    crop.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    crop.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    crop.filter(idx);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for(auto id : idx)
    {
       inliers->indices.push_back(id);
    }

    typename pcl::PointCloud<PointT>::Ptr noRoof (new pcl::PointCloud< PointT>);
    pcl::ExtractIndices< PointT> extract;
    extract.setInputCloud(cloud_filteredRegion  );
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*noRoof);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return noRoof;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices< PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr  plane_cloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr rest (new pcl::PointCloud< PointT>);
    extract.setInputCloud(cloud  );
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);
    extract.setNegative(true);
    extract.filter(*rest);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(rest, plane_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    int maxIterations, 
    float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    auto  inliers = ransacPlane(cloud, maxIterations, distanceThreshold);


    if( inliers->indices.size() == 0 )
    {
        std::cerr << "Could not estima a planar mode for the given  cloud" << std::endl;
    
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}





template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        float clusterTolerance,
        int minSize,
        int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    auto tree = new KdTree<PointT>;
    std::vector<PointT> points(cloud->points.size());
    for(int i = 0 ; i <  cloud->points.size() ; i++)
    {
        auto point = cloud->points.at(i);
        tree->insert(point , i);
        points.at(i) = point;

    }
    auto  clustersIDX = euclideanCluster(points , tree , clusterTolerance , minSize ,maxSize);
    for (int i = 0 ; i < clustersIDX.size() ; i++)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      for (int j = 0 ; j < clustersIDX.at(i).size() ; j++)
        cloud_cluster->points.push_back (cloud->points.at(clustersIDX.at(i).at(j))); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
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
