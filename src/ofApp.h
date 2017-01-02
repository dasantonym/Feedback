#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinectV2.h"
#include "ofxOpticalFlowFarneback.h"
#include "ofxGpuParticles.h"
#include "Caustics.h"
#include "ofxOsc.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();
    
    void drawFlowColored(float width, float height);
    
    void keyPressed(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    
    void onParticlesUpdate(ofShader& shader);
    void onParticlesDraw(ofShader& shader);

    ofxPanel panel;

    ofxGpuParticles particles;

    vector<shared_ptr <ofxKinectV2>> kinects;
    vector<ofTexture> texDepth;
    vector<ofTexture> texRGB;
    
    Caustics caustics;
    
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    
    ofxOpticalFlowFarneback flowSolver;
    
    ofxOscReceiver receiver;
    
    cv::Mat canny_input;
    cv::Mat canny_output;
    
    cv::Mat grayImage; // grayscale depth image
    cv::Mat grayImageAvg;
    
    cv::Mat grayThreshNear;
    cv::Mat grayThreshFar;
    
    ofImage outImage;
    ofImage inImage;
    //ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    //ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    //ofxCvContourFinder contourFinder;
    
    cv::Point2f mc;
    
    
    
    unsigned int displayList;
    
    bool bThreshWithOpenCV;
    bool bDrawPointCloud;
    
    int nearThreshold;
    int farThreshold;
    
    int angle;
    float attack;
    float amp;
    float pitch;
    
    float velocityMin;
    float velocityMax;
    float angleMin;
    float angleMax;
    
    float lineScale;
    int resolution;
    
    bool drawOutlines;
    bool drawParticles;
    bool drawCaustics;
    
    bool flipImage;
    bool flipParticles;
    
    // used for viewing the point cloud
    ofEasyCam easyCam;
};
