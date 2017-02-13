#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <functional>

#include "colorindex.h"
#include "groundfilter.h"

#include "tools.h"

//#include <pcl/io/pcd_io.h>
//#include <pcl/io/lzf_image_io.h>
//#include <pcl/common/projection_matrix.h>
//#include <pcl/console/print.h>
//#include <pcl/console/parse.h>
//#include <pcl/console/time.h>

//using namespace pcl;
//using namespace pcl::io;
//using namespace pcl::console;




int main(int argc, char *argv[])
{
    std::string names[4] = { "/home/petra/Programming/PCLGrabber/build/data/20160915T125738_copy/orig/frame_20160915T125803.510514_orig.png_",
                             "/home/petra/Programming/PCLGrabber/build/data/20160915T125738_copy/orig/frame_20160915T125746.877580_orig.png_",
                             "/home/petra/Programming/PCLGrabber/build/data/20160915T125738_copy/orig/frame_20160915T125756.077540_orig.png_",
                             "/home/petra/Programming/PCLGrabber/build/data/20160915T125738_copy/orig/frame_20160915T125807.210517_orig.png_"};


    cwd::GroundFilter AllFilters(std::bind(cwd::computeExcessGreen, std::placeholders::_1, false), true, 200);
    AllFilters.addSecondaryFunction(std::bind(cwd::computeModifiedExcessGreen, std::placeholders::_1, false));
    AllFilters.addSecondaryFunction(std::bind(cwd::computeCIVE, std::placeholders::_1, false));
    AllFilters.addSecondaryFunction(std::bind(cwd::computeVEG, std::placeholders::_1, false));
    AllFilters.addSecondaryFunction(cwd::computeCombination);
//    cv::Mat masked, binarized, histogram;
//    cv::namedWindow("Display", CV_WINDOW_NORMAL);
//
//    for (int i=0; i < 1; ++i){
//        std::string &name = names[i];
//        cv::Mat img = cv::imread(name, CV_LOAD_IMAGE_COLOR);;
//        if(img.empty())
//           return -1;
//        cv::resizeWindow("Display", img.cols/1.1, img.rows/1.1);
//
//        masked = AllFilters.processFrame(img);
//
//        cv::imshow("Display", masked);
//        cv::waitKey();
//
//        binarized = AllFilters.getBinarizationResult();
//
//        cv::imshow("Display", binarized);
//        cv::waitKey();
//
//        AllFilters.calculateHistogram(histogram);
//
//        cv::resizeWindow("Display", histogram.cols, histogram.rows);
//        cv::imshow("Display", histogram);
//        cv::waitKey();
//    }
//
//    cv::destroyWindow("Display");

    for (int i=1; i < argc; ++i){
        if (std::string(argv[i]).length() <= 5)
            continue;
        if (std::string(argv[i]).substr(std::string(argv[i]).length() -4, 4) != ".xml")
            continue;
        std::string stripEnd = std::string(argv[i]).substr(0, std::string(argv[i]).length() -4);
        std::cout << "Processing " << stripEnd << std::endl;
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        std::string rgb = stripEnd + "_rgb.pclzf";
        std::string depth = stripEnd + "_depth.pclzf";
        std::string xml = stripEnd + ".xml";
        std::string out = stripEnd + ".data";

        std::ofstream outFile(out);
        cwd::loadPCLZF(rgb, depth, xml, cloud);
        AllFilters.process3dFrame(cloud, outFile);
        outFile.close();
    }
    return 0;
}






