#ifndef GROUNDFILTER_H
#define GROUNDFILTER_H

#include <functional>

#include <opencv2/core/core.hpp>    // for pcl::Point3_

#include <pcl/point_cloud.h>        // for pcl::PointCloud
#include <pcl/point_types.h>        // for pcl::PointXYZRGBA

#include <iostream>
#include <fstream>

namespace cwd{

typedef std::function<float (const cv::Point3_<uchar> &)> rgbIndex;

class GroundFilter
{
    public:
        GroundFilter(rgbIndex indexFunction, bool highGround = false, float upperLimit = 150.f) :
            indexFunction_(indexFunction), highGround_(highGround), indexUpperLimit(upperLimit) {}
        virtual ~GroundFilter();

        const cv::Mat &processFrame(const cv::Mat &frame);

        const cv::Mat &getIndexImage() const { return indexImage; }
        const cv::Mat &getBinarizationResult() const { return binarizationResult; }

        void calculateHistogram(cv::Mat &histogram) const;

        void process3dFrame(pcl::PointCloud<pcl::PointXYZRGBA> &cloud, std::ostream &out = std::cout);

        void addSecondaryFunction(rgbIndex indexFunction) {secondaryIndex.push_back(indexFunction); }
    protected:
    private:

        rgbIndex indexFunction_;

        std::vector <rgbIndex> secondaryIndex;

        /* to process 2D rgb image */

        bool highGround_;
        float indexUpperLimit;
        cv::Mat frame_;
        cv::Mat indexImage;
        cv::Mat binarizationResult;
        cv::Mat removalResult;
        float minValue, maxValue;

        void calculateIndexImage();

        /* to process 3D point cloud */

        pcl::PointCloud<pcl::PointXYZRGBA> frame3d_;
};

}

#endif // GROUNDFILTER_H
