#pragma once

#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxGpuParticles.h"
#include "ofxGui.h"

// adds 2M particles
#define PARTICLE_GRID_X 2000
#define PARTICLE_GRID_Y 1000

#ifdef USE_OPTICAL_FLOW
#include "ofxOpticalFlowFarneback.h"
#endif

#include "OSCReceiver.h"
#include "DepthAnalysis.h"

class ofApp : public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();

    void particleSetup();
    void onParticlesUpdate(ofShader& shader);
    
    void keyPressed(int key);
    void windowResized(int w, int h);

    ofxPanel _panel;
    OSCReceiver _osc;
    DepthAnalysis _depthAnalysis;
    ofxGpuParticles _particles;

    moments_t _moments;
    analysis_config_t _analysisConfig;

    vector<shared_ptr <ofxKinectV2>> _kinect;
    vector<ofTexture> _texDepth;
    vector<ofTexture> _texRGB;

    ofVec2f _viewSize;
    GLuint _displayList;

    bool _bTexturesInitialized;
    bool _bParticlesInitialized;

    ofxToggle _optsShowMoments;
    ofxToggle _optsShowGui;
    ofxToggle _optsShowStats;
    ofxToggle _optsReceiveOSC;
    ofxToggle _optsFreezeParticles;

#ifdef USE_OPTICAL_FLOW
    ofxOpticalFlowFarneback flowSolver;

    void drawFlowColored(float width, float height, float degMin = .0f,
            float degMax = 360.f, float velMin = 2.f, float velMax = 1000.f);
#endif
};
