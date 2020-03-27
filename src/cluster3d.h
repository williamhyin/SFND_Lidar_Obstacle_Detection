//
// Created by hyin on 2020/3/25.
//

#ifndef PLAYBACK_CLUSTER3D_H
#define PLAYBACK_CLUSTER3D_H

#include <pcl/common/common.h>
#include <chrono>
#include <string>
#include "kdtree3d.h"

namespace lidar_obstacle_detection {

    // shorthand for point cloud pointer
    template<typename PointT>
    using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

    template<typename PointT>
    class ClusterPts {
    private:
        int num_points;
        float distanceTol;
        int minClusterSize;
        int maxClusterSize;
        std::vector<bool> processed;
        std::vector<PtCdtr<PointT>> clusters;

    public:
        ClusterPts(int nPts, float cTol, int minSize, int maxSize) : num_points(nPts), distanceTol(cTol),
                                                                     minClusterSize(minSize), maxClusterSize(maxSize) {
            processed.assign(num_points, false);
        }

        ~ClusterPts();

        void
        clusterHelper(int ind, PtCdtr<PointT> cloud, std::vector<int> &cluster, KdTree *tree);

        std::vector<PtCdtr<PointT>> EuclidCluster(PtCdtr<PointT> cloud);

    };
}
#endif //PLAYBACK_CLUSTER3D_H
