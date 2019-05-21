#ifndef DEPTH_GMM_H
#define DEPTH_GMM_H

#include<string.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

using namespace cv;
static const int defaultNMixtures = 5;
static const int defaultHistory = 200;
static const double defaultBackgroundRatio = 0.7;
static const double defaultVarThreshold = 100;
//static const double defaultVarThreshold = 1;
static const double defaultNoiseSigma = 0.1;
static const double defaultInitialWeight = 0.05;


template<typename VT>
struct MixData {
    float sortKey;
    float weight;
    VT mean;
    VT var;
};

class Depth_Gmm {
public:
    Depth_Gmm() {
        frameSize = cv::Size(0, 0);
        frameType = 0;

        nframes = 0;
        nmixtures = defaultNMixtures;
        history = defaultHistory;
        varThreshold = defaultVarThreshold;
        backgroundRatio = defaultBackgroundRatio;
        noiseSigma = defaultNoiseSigma;

    }

    void init(cv::Rect _box, cv::Size _frameSize, int _frameType);

    void processdepth(const cv::Mat &image, cv::Mat &fgmask, double learningRate,
                      cv::Mat &bgmodel, int nmixtures, double backgroundRatio,
                      double varThreshold, double noiseSigma);

    void run(cv::Rect _box, cv::Mat _image, cv::OutputArray _fgmask, double learningRate);

    void run(cv::Rect _box, cv::Mat _image, cv::OutputArray _fgmask, double learningRate, cv::Mat _bgmodel);

    void saveBgmodel(cv::Rect _box, cv::Mat _image, cv::OutputArray &_fgmask, double learningRate, int num,
                     cv::String save_name);

protected:
    cv::Size frameSize;
    int frameType;
    cv::Rect box;
    cv::Mat bgmodel;
    int nframes;
    int history;
    int nmixtures;
    double varThreshold;
    double backgroundRatio;
    double noiseSigma;
};

#endif // DEPTH_GMM_H
