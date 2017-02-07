#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <functional>

#include "colorindex.h"
#include "groundfilter.h"

int main(int argc, char *argv[])
{
    std::string names[4] = { "/home/petra/Programming/PCLGrabber/build/data/20160915T125738_copy/orig/frame_20160915T125803.510514_orig.png_",
                             "/home/petra/Programming/PCLGrabber/build/data/20160915T125738_copy/orig/frame_20160915T125746.877580_orig.png_",
                             "/home/petra/Programming/PCLGrabber/build/data/20160915T125738_copy/orig/frame_20160915T125756.077540_orig.png_",
                             "/home/petra/Programming/PCLGrabber/build/data/20160915T125738_copy/orig/frame_20160915T125807.210517_orig.png_"};


    cwd::GroundFilter CIVEFilter(std::bind(cwd::computeExcessGreen, std::placeholders::_1, false), true, 200);
    cv::Mat masked, binarized, histogram;
    cv::namedWindow("Display", CV_WINDOW_NORMAL);

    for (int i=0; i < 1; ++i){
        std::string &name = names[i];
        cv::Mat img = cv::imread(name, CV_LOAD_IMAGE_COLOR);;
        if(img.empty())
           return -1;
        cv::resizeWindow("Display", img.cols/1.1, img.rows/1.1);

        masked = CIVEFilter.processFrame(img);

        cv::imshow("Display", masked);
        cv::waitKey();

        binarized = CIVEFilter.getBinarizationResult();

        cv::imshow("Display", binarized);
        cv::waitKey();

        CIVEFilter.calculateHistogram(histogram);

        cv::resizeWindow("Display", histogram.cols, histogram.rows);
        cv::imshow("Display", histogram);
        cv::waitKey();
    }

    cv::destroyWindow("Display");
    return 0;
}



