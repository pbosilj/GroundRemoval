#include "groundfilter.h"

#include <iostream>
#include <algorithm>

namespace cwd{


GroundFilter::~GroundFilter()
{
    //dtor
}

const cv::Mat &GroundFilter::processFrame(const cv::Mat &frame){
    frame_ = frame;
    indexFunction_(frame_, indexImage);

    bool flag = false;
    for (int i=0; i < indexImage.rows; ++i){
        const float *inputRow = indexImage.ptr<float>(i);
        for (int j=0; j < indexImage.cols; ++j){
            if (inputRow[j] >= indexUpperLimit)
                continue;
            if (!flag){
                minValue = maxValue = inputRow[j];
                flag = true;
            }
            else{
                minValue = std::min(minValue, inputRow[j]);
                maxValue = std::max(maxValue, inputRow[j]);
            }
        }
    }

    float scale = 255.f / (maxValue - minValue);
    std::cout << "Max: " << maxValue << " Min: " << minValue << std::endl;
    indexImage.convertTo(binarizationResult, CV_8UC1, scale, -minValue*scale);

    double threshold;
    if (highGround_)
        threshold = cv::threshold(binarizationResult, binarizationResult, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    else
        threshold = cv::threshold(binarizationResult, binarizationResult, 0, 255, cv::THRESH_BINARY_INV + cv::THRESH_OTSU);

    std::cout << "Otsu\'s threshold: " << (threshold / scale + minValue ) << std::endl;

    removalResult = cv::Mat::zeros(frame_.size(), frame_.type());

    frame_.copyTo(removalResult, binarizationResult);

    return removalResult;
}

void GroundFilter::calculateHistogram(cv::Mat &histogram){
    int histSize = 256;
    //float range[] = {9.048, 9.052};
    float range[] = {minValue, maxValue};
    const float * histRange = { range } ;
    cv::Mat hist;
    cv::calcHist(&indexImage, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);
    int hist_w = 1024, hist_h = 1024;
    int bin_w = cvRound((double) hist_w / histSize );

    histogram = cv::Mat(hist_h, hist_w, CV_8UC1, cv::Scalar(0));
    cv::normalize(hist, hist, 0, indexImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
    for (int i= 1; i < histSize; ++i){
        cv::line (histogram, cv::Point (bin_w * (i-1), hist_h - cvRound(hist.at<float>(i-1)) ),
                             cv::Point (bin_w * (i)  , hist_h - cvRound(hist.at<float>(i))   ),
                             cv::Scalar(255), 2, 8, 0);
    }
}

}
