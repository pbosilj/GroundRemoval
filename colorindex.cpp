#include "colorindex.h"

namespace cwd{

void computeExcessGreen(const cv::Mat &input, cv::Mat &output){
    double _r, _g, _b;
    double r, g, b;
    output = cv::Mat(input.rows, input.cols, CV_32F, cv::Scalar::all(0));
    for (int i=0; i < input.rows; ++i){
        const cv::Point3_<uchar>* inputRow = input.ptr<cv::Point3_<uchar> >(i);
        float *outputRow = output.ptr<float>(i);
        for (int j=0; j < input.cols; ++j){
            _r = inputRow[j].x / 255.f;
            _g = inputRow[j].y / 255.f;
            _b = inputRow[j].z / 255.f;
            r = _r / (_r + _g + _b);
            g = _g / (_r + _g + _b);
            b = _b / (_r + _g + _b);
            outputRow[j] = 2*g - b - r;
        }
    }
}

void computeModifiedExcessGreen(const cv::Mat &input, cv::Mat &output){
    output = cv::Mat(input.rows, input.cols, CV_32F, cv::Scalar::all(0));
    for (int i=0; i < input.rows; ++i){
        const cv::Point3_<uchar>* inputRow = input.ptr<cv::Point3_<uchar> >(i);
        float *outputRow = output.ptr<float>(i);
        for (int j=0; j < input.cols; ++j){
            outputRow[j] = -0.884 * inputRow[j].x + 1.262 * inputRow[j].y - 0.311 * inputRow[j].z;
        }
    }
}

void computeNormalizedModifiedExcessGreen(const cv::Mat &input, cv::Mat &output){
    double _r, _g, _b;
    double r, g, b;
    output = cv::Mat(input.rows, input.cols, CV_32F, cv::Scalar::all(0));
    for (int i=0; i < input.rows; ++i){
        const cv::Point3_<uchar>* inputRow = input.ptr<cv::Point3_<uchar> >(i);
        float *outputRow = output.ptr<float>(i);
        for (int j=0; j < input.cols; ++j){
            _r = inputRow[j].x / 255.f;
            _g = inputRow[j].y / 255.f;
            _b = inputRow[j].z / 255.f;
            r = _r / (_r + _g + _b);
            g = _g / (_r + _g + _b);
            b = _b / (_r + _g + _b);
            outputRow[j] = -0.884 * r + 1.262 * g - 0.311 * b;
        }
    }
}

void computeExcessRed(const cv::Mat &input, cv::Mat &output){
    output = cv::Mat(input.rows, input.cols, CV_32F, cv::Scalar::all(0));
    for (int i=0; i < input.rows; ++i){
        const cv::Point3_<uchar>* inputRow = input.ptr<cv::Point3_<uchar> >(i);
        float *outputRow = output.ptr<float>(i);
        for (int j=0; j < input.cols; ++j){
            outputRow[j] = 1.3 * inputRow[j].x - 1 * inputRow[j].y;
        }
    }
}

void computeNormalizedExcessRed(const cv::Mat &input, cv::Mat &output){
    double _r, _g, _b;
    double r, g, b;
    output = cv::Mat(input.rows, input.cols, CV_32F, cv::Scalar::all(0));
    for (int i=0; i < input.rows; ++i){
        const cv::Point3_<uchar>* inputRow = input.ptr<cv::Point3_<uchar> >(i);
        float *outputRow = output.ptr<float>(i);
        for (int j=0; j < input.cols; ++j){
            _r = inputRow[j].x / 255.f;
            _g = inputRow[j].y / 255.f;
            _b = inputRow[j].z / 255.f;
            r = _r / (_r + _g + _b);
            g = _g / (_r + _g + _b);
            b = _b / (_r + _g + _b);
            //outputRow[j] = 1.3 * r - 1 * g;
            outputRow[j] = 1.4 * r - 1 * g;
        }
    }
}

void computeCIVE(const cv::Mat &input, cv::Mat &output){
    output = cv::Mat(input.rows, input.cols, CV_32F, cv::Scalar::all(0));
    for (int i=0; i < input.rows; ++i){
        const cv::Point3_<uchar>* inputRow = input.ptr<cv::Point3_<uchar> >(i);
        float *outputRow = output.ptr<float>(i);
        for (int j=0; j < input.cols; ++j){
            outputRow[j] = 0.441 * inputRow[j].x - 0.811 * inputRow[j].y + 0.385 * inputRow[j].z  + 18.78745;
        }
    }
}

void computeNormalizedCIVE(const cv::Mat &input, cv::Mat &output){
    double _r, _g, _b;
    double r, g, b;
    output = cv::Mat(input.rows, input.cols, CV_32F, cv::Scalar::all(0));
    for (int i=0; i < input.rows; ++i){
        const cv::Point3_<uchar>* inputRow = input.ptr<cv::Point3_<uchar> >(i);
        float *outputRow = output.ptr<float>(i);
        for (int j=0; j < input.cols; ++j){
            _r = inputRow[j].x / 255.f;
            _g = inputRow[j].y / 255.f;
            _b = inputRow[j].z / 255.f;
            r = _r / (_r + _g + _b);
            g = _g / (_r + _g + _b);
            b = _b / (_r + _g + _b);
            outputRow[j] = 0.441 * r - 0.811 * g + 0.385 * b + 18.78745;
        }
    }
}

/// \note No need for normalization - same results (tested).
void computeVEG(const cv::Mat &input, cv::Mat &output){
    double a = 0.667;
    output = cv::Mat(input.rows, input.cols, CV_32F, cv::Scalar::all(0));
    //bool flag = false;
    for (int i=0; i < input.rows; ++i){
        const cv::Point3_<uchar>* inputRow = input.ptr<cv::Point3_<uchar> >(i);
        float *outputRow = output.ptr<float>(i);
        for (int j=0; j < input.cols; ++j){
            double denominator = (std::pow(inputRow[j].x, a) * std::pow(inputRow[j].z, 1.-a));
            outputRow[j] = inputRow[j].y / ((denominator <= 1e-3)?(1e-3):(denominator));
        }
    }
}

void computeCombination(const cv::Mat &input, cv::Mat &output){ // take a look at the combination. sth's horrible
    cv::Mat ExG, CIVE, VEG;
    computeExcessGreen(input, ExG);
    computeCIVE(input, CIVE);
    computeVEG(input, VEG);

    cv::Mat temp(ExG.rows, ExG.cols, ExG.type(), cv::Scalar(0));

    cv::Mat temp2;
    cv::addWeighted(ExG, 0.36, CIVE, 0.47, 0, temp);
    cv::addWeighted(temp, 1, VEG, 0.17, 0, output);
}


}
