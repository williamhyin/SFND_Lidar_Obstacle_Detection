//
// Created by hyin on 2020/3/25.
//

#ifndef PLAYBACK_RANSAC3D_H
#define PLAYBACK_RANSAC3D_H

#include <unordered_set>
#include <pcl/common/common.h>

namespace lidar_obstacle_detection {

    // shorthand for point cloud pointer
    template<typename PointT>
    using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

    template<typename PointT>
    class Ransac {
    private:
        int maxIterations;
        float distanceTol;
        int num_points;

    public:
        Ransac(int maxIter, float distTol, int nPts) : maxIterations(maxIter), distanceTol(distTol), num_points(nPts) {}

        ~Ransac();

        std::unordered_set<int> Ransac3d(PtCdtr<PointT> cloud);
    };
}
#endif //PLAYBACK_RANSAC3D_H
