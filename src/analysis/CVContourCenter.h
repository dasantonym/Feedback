#ifndef FEEDBACK_CVCONTOURCENTER_H
#define FEEDBACK_CVCONTOURCENTER_H

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxCv.h"

typedef struct {
    vector<cv::Point> contour;
    ofVec3f center;
    int16_t index;
    float size;
} moments_t;

typedef struct {
    bool flipImage;
    bool flipCenter;
    ofVec2f offset;
    uint16_t blurAmount;
} analysis_config_t;

class CVContourCenter: public ofThread {
public:
    CVContourCenter();

    void init(uint16_t width, uint16_t height);
    void update(ofTexture *texture);
    void threadedFunction();

    bool isUpdated();

    analysis_config_t getConfig();
    moments_t getMoments();

    ofxToggle optsFlipImage;
    ofxToggle optsFlipCenter;

private:
    bool _bTexturesInitialized;
    bool _bUpdated;
    bool _bDirty;

    cv::Mat _grayImage;
    cv::Mat _grayImageAvg;

    uint16_t _width;
    uint16_t _height;

    cv::Moments _moment;
    ofVec3f _momentCenter;
    int16_t _momentIndex;
    float _momentSize;

    ofVec2f _offset;

    vector<vector<cv::Point> > _contours;
    vector<cv::Vec4i> _hierarchy;

    uint16_t _blurAmount;
    ofPixels _depthPixels;
};


#endif //FEEDBACK_CVCONTOURCENTER_H
