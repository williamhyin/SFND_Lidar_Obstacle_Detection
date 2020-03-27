// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <unordered_set>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"

#include "ransac3d.cpp"
#include "cluster3d.cpp"

namespace lidar_obstacle_detection {

    // shorthand for point cloud pointer
    template<typename PointT>
    using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

    template<typename PointT>

    class ProcessPointClouds {
    public:

        //constructor
        ProcessPointClouds();

        //deconstructor
        ~ProcessPointClouds();

        void numPoints(PtCdtr<PointT> cloud);

        PtCdtr<PointT>FilterCloud(PtCdtr<PointT> cloud, float filterRes, Eigen::Vector4f minPoint,Eigen::Vector4f maxPoint);

        std::pair<PtCdtr<PointT>, PtCdtr<PointT>>SeparateClouds(pcl::PointIndices::Ptr inliers, PtCdtr<PointT> cloud);

        std::pair<PtCdtr<PointT>, PtCdtr<PointT>>RansacSegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceTol);

        std::pair<PtCdtr<PointT>, PtCdtr<PointT>>SegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceTol);

        std::vector<PtCdtr<PointT>>EuclideanClustering(PtCdtr<PointT> cloud, float clusterTolerance, int minSize,int maxSize);

        std::vector<PtCdtr<PointT>>Clustering(PtCdtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize);

        Box BoundingBox(PtCdtr<PointT> cluster);

        void savePcd(PtCdtr<PointT> cloud, std::string file);

        PtCdtr<PointT> loadPcd(std::string file);

        std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    };
}
#endif /* PROCESSPOINTCLOUDS_H_ */