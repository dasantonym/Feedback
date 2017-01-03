#ifndef FEEDBACK_OPTICALFLOW_H
#define FEEDBACK_OPTICALFLOW_H

#include "ofMain.h"
#include "ofxOpticalFlowFarneback.h"

typedef struct {
    float degMean;
    ofVec2f velMin;
    ofVec2f velMax;

} flow_results_t;

class OpticalFlow: public ofThread {
public:
    void setup(uint16_t width, uint16_t height);
    void update(ofTexture *texture, bool buildMesh = false);
    void threadedFunction();

    ofMesh getVelocityMesh();
    flow_results_t getFlowResults();

private:
    void analyzeFlowResults(ofVec2f degLimits = ofVec2f(.0f, 360.f),
            ofVec2f velLimits = ofVec2f(2.f, 1000.f), ofVec2f skipPixels = ofVec2f(3.f, 3.f));

    bool _buildMesh;

    float _degMean;
    ofVec2f _velMin;
    ofVec2f _velMax;

    ofPixels _depthPixels;
    ofxCvGrayscaleImage _depthImage;
    ofxOpticalFlowFarneback _flowSolver;
    ofMesh _velMesh;

    uint16_t _width;
    uint16_t _height;
};


#endif //FEEDBACK_OPTICALFLOW_H
