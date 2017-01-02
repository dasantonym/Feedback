#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinectV2.h"
#include "ofxOpticalFlowFarneback.h"
#include "ofxGpuParticles.h"
#include "ofxOsc.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();
    
    void drawFlowColored(float width, float height);
    void particleSetup();
    
    void keyPressed(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    
    void onParticlesUpdate(ofShader& shader);

    ofxPanel panel;

    ofxGpuParticles particles;

    vector<shared_ptr <ofxKinectV2>> kinects;
    vector<ofTexture> texDepth;
    vector<ofTexture> texRGB;
    
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    
    ofxOpticalFlowFarneback flowSolver;
    
    ofxOscReceiver receiver;
    
    cv::Mat grayImage;
    cv::Mat grayImageAvg;
    ofImage outImage;
    
    cv::Point2f mc;
    GLuint displayList;

    bool bTexturesInitialized;

    ofVec3f _mouse;

    float attack;
    float amp;
    float pitch;
    
    float velocityMin;
    float velocityMax;
    float angleMin;
    float angleMax;
    
    float lineScale;
    uint16_t resolution;
    
    bool bShowMoments;
    bool bShowParticles;
    bool bShowGui;
    bool bUseOpticalFlow;
    
    bool bFlipImage;
    bool bFlipParticles;
};
