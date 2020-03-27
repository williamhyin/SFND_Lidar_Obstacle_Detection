//
// Created by hyin on 2020/3/25.
//

#include "cluster3d.h"

using namespace lidar_obstacle_detection;

template<typename PointT>
ClusterPts<PointT>::~ClusterPts() {}

template<typename PointT>
void ClusterPts<PointT>::clusterHelper(int ind, PtCdtr<PointT> cloud, std::vector<int> &cluster, KdTree *tree) {
    processed[ind] = true;
    cluster.push_back(ind);

    std::vector<int> nearest_point = tree->search(cloud->points[ind], distanceTol);
    for (int nearest_id:nearest_point) {
        if (!processed[nearest_id]) {
            clusterHelper(nearest_id, cloud, cluster, tree);
        }
    }

}

template<typename PointT>
std::vector<PtCdtr<PointT>> ClusterPts<PointT>::EuclidCluster(PtCdtr<PointT> cloud) {
    KdTree *tree = new KdTree;
    for (int ind = 0; ind < num_points; ind++) {
        tree->insert(cloud->points[ind], ind);
    }
    for (int ind = 0; ind < num_points; ind++) {
        if (processed[ind]) {
            ind++;
            continue;
        }
        std::vector<int> cluster_ind;
        PtCdtr<PointT> cloudCluster(new pcl::PointCloud<PointT>);
        clusterHelper(ind, cloud, cluster_ind, tree);

        int cluster_size = cluster_ind.size();
        if (cluster_size >= minClusterSize && cluster_size <= maxClusterSize) {
            for (int i = 0; i < cluster_size; i++) {
                cloudCluster->points.push_back(cloud->points[cluster_ind[i]]);
            }
            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            clusters.push_back(cloudCluster);
        }
    }
    return clusters;
}