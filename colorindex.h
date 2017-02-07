#ifndef COLORINDEX_H
#define COLORINDEX_H

#include <opencv2/core/core.hpp>

namespace cwd{

float computeExcessGreen(const cv::Point3_<uchar> &rgb, bool normalize = false);
float computeModifiedExcessGreen(const cv::Point3_<uchar> &rgb, bool normalize = false);
float computeExcessRed(const cv::Point3_<uchar> &rgb, bool normalize = false);
float computeCIVE(const cv::Point3_<uchar> &rgb, bool normalize = false);
float computeVEG(const cv::Point3_<uchar> &rgb, bool normalize = false);
float computeCombination(const cv::Point3_<uchar> &rgb);

}

#endif // COLORINDEX_H
