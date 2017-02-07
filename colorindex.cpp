#include "colorindex.h"

#include <iostream>

namespace cwd{

void normalizeColorCoords(float &r, float &g, float &b){
    double sum = r + g + b;
    r /= sum;
    g /= sum;
    b /= sum;
}

float computeExcessGreen(const cv::Point3_<uchar> &rgb, bool normalize){
    float _r = rgb.x, _g = rgb.y, _b = rgb.z;
    if (normalize) normalizeColorCoords(_r, _g, _b);
    return 2*_g - _b - _r;
}

float computeExcessRed(const cv::Point3_<uchar> &rgb, bool normalize){
    float _r = rgb.x, _g = rgb.y, _b = rgb.z;
    if (normalize) normalizeColorCoords(_r, _g, _b);
    return 1.3 * _r - 1 * _g;
}

float computeCIVE(const cv::Point3_<uchar> &rgb, bool normalize){
    float _r = rgb.x, _g = rgb.y, _b = rgb.z;
    if (normalize) normalizeColorCoords(_r, _g, _b);
    return 0.441 * _r - 0.811 * _g + 0.385 * _b + 18.78745;
}

float computeVEG(const cv::Point3_<uchar> &rgb, bool normalize){
    float _r = rgb.x, _g = rgb.y, _b = rgb.z;
    double a = 0.667;
    double denominator = (std::pow(_r, a) * std::pow(_b, 1.-a));
    return _g / ((denominator <= 1e-3)?(1e-3):(denominator));
}


float computeCombination(const cv::Point3_<uchar> &rgb){
    float _r = rgb.x, _g = rgb.y, _b = rgb.z;
    return 0.36 * computeExcessGreen(rgb, true) +
           0.47 * computeCIVE(rgb, false) +
           0.17 * computeVEG(rgb);
}

}
