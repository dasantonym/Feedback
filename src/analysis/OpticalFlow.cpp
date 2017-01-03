#include "OpticalFlow.h"

void OpticalFlow::setup(uint16_t width, uint16_t height) {
    _width = width;
    _height = height;

    _flowSolver.setup(_width, _height, .35f, 5, 10, 1, 3, 2.25f, false, false);
}

void OpticalFlow::update(ofTexture *texture, bool buildMesh) {
    lock();
    _buildMesh = buildMesh;
    unlock();

    if (isThreadRunning()) return;

    if (texture->isAllocated()) {
        lock();
        texture->readToPixels(_depthPixels);
        unlock();
    }
    startThread(false);
}

void OpticalFlow::threadedFunction() {
    _depthImage.setUseTexture(false);
    _depthImage.setFromPixels(_depthPixels);
    _flowSolver.update(_depthImage);

    lock();
    analyzeFlowResults();
    unlock();
}

ofMesh OpticalFlow::getVelocityMesh() {
    ofMesh mesh;
    lock();
    mesh.append(_velMesh);
    unlock();
    return mesh;
}

flow_results_t OpticalFlow::getFlowResults() {
    flow_results_t results;
    lock();
    results.degMean = _degMean;
    results.velMax.x = _velMax.x;
    results.velMax.y = _velMax.y;
    results.velMin.x = _velMin.x;
    results.velMin.y = _velMin.y;
    unlock();
    return results;
}

void OpticalFlow::analyzeFlowResults(ofVec2f degLimits, ofVec2f velLimits, ofVec2f skipPixels) {
    ofPoint vel;
    float angle;

    if (_buildMesh) {
        _velMesh.clear();
        _velMesh.disableTextures();
        _velMesh.setMode(OF_PRIMITIVE_LINES);
    }

    for(int x=0; x<_width; x+=skipPixels.x) {
        for(int y=0; y<_height; y+=skipPixels.y) {
            vel = _flowSolver.getVelAtPixel(x, y);
            ofVec3f p(x,y);
            ofVec3f vc = vel;
            vc.normalize();
            angle = atan2(vc.y, vc.x)+3.14159265f;
            if(vel.length() < velLimits.x || vel.length() >  velLimits.y
                    || angle < ofDegToRad(degLimits.x)
                    || angle > ofDegToRad(degLimits.y)) {
                continue;
            }
            float angleDeg = angle/(3.14159265f*2.0f);

            if (_buildMesh) {
                ofFloatColor c;
                c.setHsb(angleDeg, 1.f, 1.f);
                c.a = 0.25f;
                _velMesh.addColor(c);
                _velMesh.addVertex(p);
                c.a = 0.f;
                _velMesh.addColor(c);
                _velMesh.addVertex(p + vel * 10.f);
            }
        }
    }
}