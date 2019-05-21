#include <fstream>
#include"Depth_Gmm.h"
#include<iostream>

using namespace std;

//static const int defaultNMixtures = 5;
//static const int defaultHistory = 200;
//static const double defaultBackgroundRatio = 0.7;
//static const double defaultVarThreshold = 100;
////static const double defaultVarThreshold = 1;
//static const double defaultNoiseSigma = 0.1;
//static const double defaultInitialWeight = 0.05;



void Depth_Gmm::init(Rect _box, Size _frameSize, int _frameType) {

    frameSize = _frameSize;
    frameType = _frameType;
    nframes = 0;
    box = _box;
    int nchannels = CV_MAT_CN(frameType);
    CV_Assert(CV_MAT_DEPTH(frameType) == CV_32F);

    // for each gaussian mixture of each pixel bg model we store ...
    // the mixture sort key (w/sum_of_variances), the mixture weight (w),
    // the mean (nchannels values) and
    // the diagonal covariance matrix (another nchannels values)
    bgmodel.create(1, frameSize.height * frameSize.width * nmixtures * (2 + 2 * nchannels), CV_32F);
    bgmodel = Scalar::all(0);

}

void Depth_Gmm::processdepth(const Mat &image, Mat &fgmask, double learningRate,
                             Mat &bgmodel, int nmixtures, double backgroundRatio,
                             double varThreshold, double noiseSigma) {
    int x, y, k, k1;
    float alpha = (float) learningRate, T = (float) backgroundRatio, vT = (float) varThreshold;
    int K = nmixtures;
    MixData<float> *mptr = (MixData<float> *) bgmodel.data;

    const float w0 = (float) defaultInitialWeight;//0.05
    const float sk0 = (float) (w0 / (defaultNoiseSigma * 2));
    const float var0 = (float) (defaultNoiseSigma * defaultNoiseSigma * 4);
    const float minVar = (float) (noiseSigma * noiseSigma);
//    cout<<"box.width"<<box.width<<endl;
//    cout<<"box.height"<<box.height<<endl;
    for (y = box.y; y < box.height; y++) {
        const float *src = image.ptr<float>(y);
        uchar *dst = fgmask.ptr<uchar>(y);

        if (alpha > 0) {
            for (x = box.x; x < box.width; x++, mptr += K) {
                float wsum = 0;
                float pix = src[x];
//                cout<<pix<<endl;
                int kHit = -1, kForeground = -1;

                for (k = 0; k < K; k++) {
                    float w = mptr[k].weight;
                    wsum += w;
                    if (w < FLT_EPSILON)
                        break;
                    float mu = mptr[k].mean;
                    float var = mptr[k].var;
                    float diff = pix - mu;
                    float d2 = diff * diff;
                    if (d2 < var) {
                        wsum -= w;
                        float dw = alpha * (1.f - w);
                        mptr[k].weight = w + dw;
                        mptr[k].mean = mu + alpha * diff;
                        var = std::max(var + alpha * (d2 - var), minVar);
                        mptr[k].var = var;
                        mptr[k].sortKey = mptr[k].weight / std::sqrt(var);

                        for (k1 = k - 1; k1 >= 0; k1--) {
                            if (mptr[k1].sortKey >= mptr[k1 + 1].sortKey)
                                break;
                            std::swap(mptr[k1], mptr[k1 + 1]);
                        }

                        kHit = k1 + 1;
                        break;
                    }
                }

                if (kHit <
                    0) // no appropriate gaussian mixture found at all, remove the weakest mixture and create a new one
                {
                    kHit = k = std::min(k, K - 1);
                    wsum += w0 - mptr[k].weight;
                    mptr[k].weight = w0;
                    mptr[k].mean = pix;
                    mptr[k].var = var0;
                    mptr[k].sortKey = sk0;
                } else
                    for (; k < K; k++)
                        wsum += mptr[k].weight;

                float wscale = 1.f / wsum;
                wsum = 0;
                for (k = 0; k < K; k++) {
                    wsum += mptr[k].weight *= wscale;
                    mptr[k].sortKey *= wscale;
                    if (wsum > T && kForeground < 0)
                        kForeground = k + 1;
                }

                dst[x] = (uchar) (-(kHit >= kForeground));
            }
        } else {
            for (x = box.x; x < box.width; x++, mptr += K) {
                float pix = src[x];
                int kHit = -1, kForeground = -1;

                for (k = 0; k < K; k++) {
                    if (mptr[k].weight < FLT_EPSILON)
                        break;
                    float mu = mptr[k].mean;
                    float var = mptr[k].var;
                    float diff = pix - mu;
                    float d2 = diff * diff;
                    if (d2 < 1.0 * vT * var) {
                        kHit = k;
                        break;
                    }
                }

                if (kHit >= 0) {
                    float wsum = 0;
                    for (k = 0; k < K; k++) {
                        wsum += mptr[k].weight;
                        if (wsum > T) {
                            kForeground = k + 1;
                            break;
                        }
                    }
                }

                dst[x] = (uchar) (kHit < 0 || kHit >= kForeground ? 255 : 0);
            }
        }
    }
}

void
Depth_Gmm::saveBgmodel(Rect _box, Mat _image, OutputArray &_fgmask, double learningRate, int num, String save_name) {

    bool needToInitialize =
            nframes == 0 || learningRate >= 1 || _image.size() != frameSize || _image.type() != frameType;

    if (needToInitialize)
        init(_box, _image.size(), _image.type());

    CV_Assert(_image.depth() == CV_32F);
    _fgmask.create(_image.size(), CV_8U);
    Mat fgmask = _fgmask.getMat();


    ++nframes;
    if (nframes < num) {
        learningRate = learningRate >= 0 && nframes > 1 ? learningRate : 1. / std::min(nframes, history);
        CV_Assert(learningRate >= 0);
    } else if (nframes == num) {
        learningRate = -1.0;
        ofstream outfile;
        outfile.open(save_name);
        MixData<float> *mptr = (MixData<float> *) bgmodel.data;
        if (outfile.is_open()) {
            for (int y = box.y; y < box.height; y++) {
                for (int x = box.x; x < box.width; x++, mptr += 5) {

                    for (int k = 0; k < 5; k++) {
                        float w = mptr[k].weight;
                        float mu = mptr[k].mean;
                        float var = mptr[k].var;
                        float s = mptr[k].sortKey;
                        outfile << w << " " << mu << " " << var << " " << s << " ";
                    }
                }
            }
        }
        std::cout << "out success" << std::endl;
    } else {
        learningRate = -1.0;
    }

    if (_image.type() == CV_32FC1)
        processdepth(_image, fgmask, learningRate, bgmodel, nmixtures, backgroundRatio, varThreshold, noiseSigma);
    else
        CV_Error(Error::StsUnsupportedFormat, "Only CV_32FC1 images are supported ");
}

void Depth_Gmm::run(Rect _box, Mat _image, OutputArray _fgmask, double learningRate) {

    bool needToInitialize =
            nframes == 0 || learningRate >= 1 || _image.size() != frameSize || _image.type() != frameType;

    if (needToInitialize)
        init(_box, _image.size(), _image.type());

    CV_Assert(_image.depth() == CV_32F);
    _fgmask.create(_image.size(), CV_8U);
    Mat fgmask = _fgmask.getMat();


    ++nframes;
    if (nframes < 30) {
        learningRate = learningRate >= 0 && nframes > 1 ? learningRate : 1. / std::min(nframes, history);
        CV_Assert(learningRate >= 0);
    } else if (nframes == 30) {
        learningRate = -1.0;
        ofstream outfile;
        outfile.open("bgmodel.txt");
        MixData<float> *mptr = (MixData<float> *) bgmodel.data;
        if (outfile.is_open()) {
            for (int y = box.y; y < box.height; y++) {
                for (int x = box.x; x < box.width; x++, mptr += 5) {

                    for (int k = 0; k < 5; k++) {
                        float w = mptr[k].weight;
                        float mu = mptr[k].mean;
                        float var = mptr[k].var;
                        float s = mptr[k].sortKey;
                        outfile << w << " " << mu << " " << var << " " << s << " ";
                    }
                }
            }
        }
        std::cout << "out success" << std::endl;
    } else {
        learningRate = -1.0;
    }

    if (_image.type() == CV_32FC1)
        processdepth(_image, fgmask, learningRate, bgmodel, nmixtures, backgroundRatio, varThreshold, noiseSigma);
    else
        CV_Error(Error::StsUnsupportedFormat, "Only CV_32FC1 images are supported ");
}

void Depth_Gmm::run(Rect _box, Mat _image, OutputArray _fgmask, double learningRate, Mat _bgmodel) {
    bool needToInitialize =
            nframes == 0 || learningRate >= 1 || _image.size() != frameSize || _image.type() != frameType;

    if (needToInitialize) {
        init(_box, _image.size(), _image.type());
        bgmodel = _bgmodel.clone();
    }

    CV_Assert(_image.depth() == CV_32F);
    _fgmask.create(_image.size(), CV_8U);
    Mat fgmask = _fgmask.getMat();
    learningRate = -1.0;
    ++nframes;

    if (_image.type() == CV_32FC1)
        processdepth(_image, fgmask, learningRate, bgmodel, nmixtures, backgroundRatio, varThreshold, noiseSigma);
    else
        CV_Error(Error::StsUnsupportedFormat, "Only CV_32FC1 images are supported ");
}
