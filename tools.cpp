#include "tools.h"

#include <pcl/io/lzf_image_io.h>        // for pcl::io::LZFRGB24ImageReader, pcl::io::LZFBayer8ImageReader, pcl::io::LZFYUV422ImageReader, pcl::io::LZFDepth16ImageReader

namespace cwd{
bool loadPCLZF (const std::string &filename_rgb,
       const std::string &filename_depth,
       const std::string &filename_params,
       pcl::PointCloud<pcl::PointXYZRGBA> &cloud){

    pcl::io::LZFRGB24ImageReader rgb;
    pcl::io::LZFBayer8ImageReader bayer;
    pcl::io::LZFYUV422ImageReader yuv;
    pcl::io::LZFDepth16ImageReader depth;

    rgb.readParameters (filename_params);
    bayer.readParameters (filename_params);
    depth.readParameters (filename_params);
    yuv.readParameters (filename_params);

    if (!rgb.read (filename_rgb, cloud))
        if (!yuv.read (filename_rgb, cloud))
            bayer.read (filename_rgb, cloud);

    depth.read (filename_depth, cloud);

    return (true);
}

}
