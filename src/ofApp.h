#pragma once

#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxGpuParticles.h"
#include "ofxGui.h"

// adds 2M particles
#define PARTICLE_GRID_X 2000
#define PARTICLE_GRID_Y 1000

#include "OSCReceiver.h"
#include "CVContourCenter.h"
#include "OpticalFlow.h"

class ofApp : public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();

    void particleSetup();
    void onParticlesUpdate(ofShader& shader);
    void onParticlesDraw(ofShader& shader);
    
    void keyPressed(int key);
    void windowResized(int w, int h);

    ofxPanel _panel;
    OSCReceiver _osc;
    CVContourCenter _cvContourCenter;
    OpticalFlow _opticalFlow;
    ofxGpuParticles _particles;

    moments_t _moments;
    analysis_config_t _analysisConfig;
    flow_results_t _flowResults;

    vector<shared_ptr <ofxKinectV2>> _kinect;
    vector<ofTexture> _texDepth;
    vector<ofTexture> _texRGB;

    ofVec2f _viewSize;
    GLuint _displayList;

    bool _bTexturesInitialized;
    bool _bParticlesInitialized;

    float _particleEnergy;
    double _particleStartTime;

    ofxToggle _optsShowMoments;
    ofxToggle _optsShowGui;
    ofxToggle _optsShowStats;
    ofxToggle _optsReceiveOSC;
    ofxToggle _optsFreezeParticles;
};
