#ifndef COLORINDEX_H
#define COLORINDEX_H

#include <opencv2/core/core.hpp>

namespace cwd{

void computeExcessGreen(const cv::Mat &input, cv::Mat &output);
void computeModifiedExcessGreen(const cv::Mat &input, cv::Mat &output);
void computeNormalizedModifiedExcessGreen(const cv::Mat &input, cv::Mat &output);
void computeExcessRed(const cv::Mat &input, cv::Mat &output);
void computeNormalizedExcessRed(const cv::Mat &input, cv::Mat &output);
void computeCIVE(const cv::Mat &input, cv::Mat &output);
void computeNormalizedCIVE(const cv::Mat &input, cv::Mat &output);
void computeVEG(const cv::Mat &input, cv::Mat &output);
void computeCombination(const cv::Mat &input, cv::Mat &output);

}

#endif // COLORINDEX_H
