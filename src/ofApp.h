#pragma once

// #define USE_OPTICAL_FLOW

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinectV2.h"
#include "ofxGpuParticles.h"
#include "ofxGui.h"

#ifdef USE_OPTICAL_FLOW
#include "ofxOpticalFlowFarneback.h"
#endif

#include "OSCReceiver.h"

class ofApp : public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();

    void particleSetup();
    void onParticlesUpdate(ofShader& shader);
    
    void keyPressed(int key);

    ofxPanel _panel;
    OSCReceiver _osc;
    ofxGpuParticles _particles;

    vector<shared_ptr <ofxKinectV2>> _kinect;
    vector<ofTexture> _texDepth;
    vector<ofTexture> _texRGB;
    
    cv::Mat _grayImage;
    cv::Mat _grayImageAvg;
    ofImage _depthImage;

    vector<vector<cv::Point> > _contours;
    vector<cv::Vec4i> _hierarchy;
    ofVec3f _momentCenter;
    GLuint _displayList;

    bool _bTexturesInitialized;
    bool _bShowMoments;
    bool _bShowParticles;
    bool _bShowGui;
    bool _bReceiveOSC;
    bool _bFlipImage;
    bool _bFlipParticles;

#ifdef USE_OPTICAL_FLOW
    ofxOpticalFlowFarneback flowSolver;

    void drawFlowColored(float width, float height, float degMin = .0f,
            float degMax = 360.f, float velMin = 2.f, float velMax = 1000.f);
#endif
};
