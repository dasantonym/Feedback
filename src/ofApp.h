#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxKinectV2.h"
#include "ofxOpticalFlowFarneback.h"
#include "ofxGpuParticles.h"
#include "Caustics.h"
#include "ofxOsc.h"

#define USE_KINECT2 false

// Windows users:
// You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:
//
//     ofxKinect/libs/libfreenect/platform/windows/inf
//
// This should install the Kinect camera, motor, & audio drivers.
//
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager.
//
// No way around the Windows driver dance, sorry.

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class ofApp : public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();
    
    void drawFlowColored();
    void drawPointCloud();
    
    void keyPressed(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    
    void onParticlesUpdate(ofShader& shader);
    void onParticlesDraw(ofShader& shader);
    
    ofxGpuParticles particles;
    
#ifdef USE_KINECT2
    ofxKinectV2 kinect2;
#endif
    
    ofxKinect kinect;
    ofTexture texDepth;
    
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
