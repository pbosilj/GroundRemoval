#ifndef TOOLS_H
#define TOOLS_H

#include <pcl/point_types.h>        // for pcl::PointXYZRGBA
#include <pcl/point_cloud.h>        // for pcl::PointCloud

namespace cwd{

bool loadPCLZF (const std::string &filename_rgb,
           const std::string &filename_depth,
           const std::string &filename_params,
           pcl::PointCloud<pcl::PointXYZRGBA> &cloud);

}

#endif // TOOLS_H
