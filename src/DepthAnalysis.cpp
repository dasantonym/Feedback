#include "DepthAnalysis.h"

using namespace cv;
using namespace ofxCv;

DepthAnalysis::DepthAnalysis() {
    optsFlipImage.setup("flipImage", false);
    optsFlipCenter.setup("flipCenter", false);
}

void DepthAnalysis::init(uint16_t width, uint16_t height) {
    _width = width;
    _height = height;

    _grayImage = Mat::zeros(_height, _width, CV_8UC1);
    _grayImageAvg = Mat::zeros(_height, _width, CV_32FC1);

    _blurAmount = 6;

    _bTexturesInitialized = true;
}

void DepthAnalysis::update(ofTexture *texture) {
    if (isThreadRunning()) return;
    if (texture->isAllocated()) {
        lock();
        texture->readToPixels(_depthPixels);
        _bUpdated = _depthPixels.isAllocated();
        unlock();
    }
    startThread(false);
}

bool DepthAnalysis::isUpdated() {
    lock();
    bool updated = _bDirty;
    unlock();
    return updated;
}

void DepthAnalysis::threadedFunction() {
    if (_bTexturesInitialized && _bUpdated) {
        bool updated = false;

        lock();
        _bUpdated = false;
        _grayImage = toCv(_depthPixels);


        if (optsFlipImage) flip(_grayImage, _grayImage, 1);

        blur(_grayImage, _grayImage, _blurAmount);

        accumulateWeighted(_grayImage, _grayImageAvg, .4f);
        _grayImageAvg.convertTo(_grayImage, CV_8UC1);

        float scale = ofGetWidth() / _width;

        _offset.y = roundf(((_height * scale) - ofGetHeight()) * .5f);
        findContours(_grayImage, _contours, _hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);

        int16_t largeIndex = -1;
        double largeSize = 0.;

        for (uint16_t i = 0; i < _contours.size(); i++) {
            double areaSize = contourArea(_contours[i]);
            if (areaSize > largeSize) {
                updated = true;
                largeSize = areaSize;
                largeIndex = i;
            }
        }

        if (largeIndex > 0) {
            _momentIndex = largeIndex;
            _momentSize = (float) largeSize;
            _moment = moments(_contours[_momentIndex]);
            _momentCenter.x = (float) (_moment.m10 / _moment.m00) * scale - .5f * ofGetWidth();
            _momentCenter.y = (float) (_moment.m01 / _moment.m00) * scale - .5f * ofGetHeight() - _offset.y;

            if (optsFlipCenter) _momentCenter.x *= -1.f;
        }

        _bDirty = _bDirty ? _bDirty : updated;
        unlock();
    }
}

analysis_config_t DepthAnalysis::getConfig() {
    analysis_config_t config;
    lock();
    config.blurAmount = _blurAmount;
    config.flipCenter = optsFlipCenter;
    config.flipImage = optsFlipImage;
    config.offset.y = _offset.y;
    unlock();
    return config;
}

moments_t DepthAnalysis::getMoments() {
    moments_t moments;
    lock();
    moments.center = _momentCenter;
    moments.contour = _contours[_momentIndex];
    moments.index = _momentIndex;
    moments.size = _momentSize;
    _bDirty = false;
    unlock();
    return moments;
}