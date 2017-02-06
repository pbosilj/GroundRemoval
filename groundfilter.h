#ifndef GROUNDFILTER_H
#define GROUNDFILTER_H

#include <functional>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cwd{

class GroundFilter
{
    public:
        GroundFilter(std::function<void (const cv::Mat &, cv::Mat &)> indexFunction, bool highGround = false, float upperLimit = 150.f) :
            indexFunction_(indexFunction), highGround_(highGround), indexUpperLimit(upperLimit) {}
        virtual ~GroundFilter();

        const cv::Mat &processFrame(const cv::Mat &frame);

        const cv::Mat &getIndexImage() { return indexImage; }
        const cv::Mat &getBinarizationResult() { return binarizationResult; }

        void calculateHistogram(cv::Mat &histogram);
    protected:
    private:
        std::function <void (const cv::Mat &, cv::Mat &)> indexFunction_;
        bool highGround_;
        float indexUpperLimit;
        cv::Mat frame_;
        cv::Mat indexImage;
        cv::Mat binarizationResult;
        cv::Mat removalResult;
        float minValue, maxValue;

};

}

#endif // GROUNDFILTER_H
