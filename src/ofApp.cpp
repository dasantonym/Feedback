#include "ofApp.h"

using namespace cv;
using namespace ofxCv;

//--------------------------------------------------------------
void ofApp::setup() {
    ofSetLogLevel(OF_LOG_NOTICE);

    _bShowParticles = false;
    _bShowMoments = false;
    _bShowGui = false;
    _bReceiveOSC = false;

    _bFlipImage = true;
    _bFlipParticles = true;

    _panel.setup("", "settings.xml", 20, 20);

    ofxKinectV2 tmp;
    vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();
    _kinect.resize(deviceList.size());
    _texDepth.resize(_kinect.size());
    _texRGB.resize(_kinect.size());

    for (uint8_t d = 0; d < _kinect.size(); d++) {
        _kinect[d] = shared_ptr <ofxKinectV2> (new ofxKinectV2());
        _kinect[d]->open(deviceList[d].serial);
        _panel.add(_kinect[d]->params);
    }

    _depthImage.setImageType(OF_IMAGE_GRAYSCALE);
    _displayList = glGenLists(1);

    particleSetup();
    ofAddListener(_particles.updateEvent, this, &ofApp::onParticlesUpdate);

    _osc.setup();

    _panel.loadFromFile("settings.xml");
}

void ofApp::onParticlesUpdate(ofShader& shader)
{
    float sq = 200.f * 200.f;
    shader.setUniform3fv("center", _momentCenter.getPtr());
    shader.setUniform1f("elapsed", (float) ofGetLastFrameTime());
    shader.setUniform1f("radiusSquared", sq);
}

//--------------------------------------------------------------
void ofApp::update() {
    for(uint8_t d = 0; d < _kinect.size(); d++){
        _kinect[d]->update();
        if(_kinect[d]->isFrameNew()){
            _texDepth[d].loadData(_kinect[d]->getDepthPixels());
            _texRGB[d].loadData(_kinect[d]->getRgbPixels());

            if (d == 0 && !_bTexturesInitialized) {
                uint16_t depthWidth = (uint16_t)_texDepth[0].getWidth();
                uint16_t depthHeight = (uint16_t)_texDepth[0].getHeight();

                _depthImage.allocate(depthWidth, depthHeight, OF_IMAGE_GRAYSCALE);
                _grayImage = Mat::zeros(depthHeight, depthWidth, CV_8UC1);
                _grayImageAvg = Mat::zeros(depthHeight, depthWidth, CV_32FC1);

#ifdef USE_OPTICAL_FLOW
                flowSolver.setup(depthWidth, depthHeight, .35f, 5, 10, 1, 3, 2.25f, false, false);
#endif

                _bTexturesInitialized = true;
            }

            if (d == 0) {
                ofPixels depthPixels;
                _texDepth[d].readToPixels(depthPixels);
                _depthImage.setFromPixels(depthPixels);
                _grayImage = toCv(_depthImage);

                if (_bFlipImage) cv::flip(_grayImage, _grayImage, 1);

                if (_bReceiveOSC) {
                    blur(_grayImage, _grayImage, (uint8_t) roundf(min(12.f * _osc.amp, 1.f)));
                } else {
                    blur(_grayImage, _grayImage, 4);
                }

                cv::accumulateWeighted(_grayImage, _grayImageAvg, .2f);
                _grayImageAvg.convertTo(_grayImage, CV_8UC1);

                _depthImage.update();

                float scale = ofGetWidth() / _depthImage.getWidth();
                int16_t yoffset = (int16_t)roundf(((_depthImage.getHeight() * scale) - ofGetHeight()) * .5f);

                cv::findContours(_grayImage, _contours, _hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);

                int16_t largeIndex = -1;
                double largeSize = 0.;

                for (uint16_t i = 0; i < _contours.size(); i++)
                {
                    double areaSize = contourArea(_contours[i]);
                    if (areaSize > largeSize)
                    {
                        largeSize = areaSize;
                        largeIndex = i;
                    }
                }

                if (largeIndex > 0)
                {
                    Moments largeMoments = moments(_contours[largeIndex]);
                    _momentCenter = ofVec3f(
                            (float)(largeMoments.m10/largeMoments.m00) * scale - .5f * ofGetWidth(),
                            (float)(largeMoments.m01/largeMoments.m00) * scale - .5f * ofGetHeight() - yoffset,
                    .0f);

                    if (_bFlipParticles) _momentCenter.x *= -1.f;

                    if (_bShowMoments) {
                        glNewList(_displayList, GL_COMPILE);
                        glColor4f(1.f, 1.f, 1.f, .5f);

                        glBegin(GL_LINE_STRIP);
                        for (uint16_t cn = 0; cn < _contours[largeIndex].size(); cn++) {
                            glVertex3d(float(_contours[largeIndex][cn].x) * scale, float(_contours[largeIndex][cn].y) * scale - yoffset, 0.f);
                        }
                        glVertex3d(float(_contours[largeIndex][0].x) * scale, float(_contours[largeIndex][0].y) * scale - yoffset, 0.f);
                        glEnd();

                        glEndList();
                    }
                }
            }
        }
    }

#ifdef USE_OPTICAL_FLOW
    if (_bTexturesInitialized) {
        flowSolver.update(_depthImage);
    }
#endif

    if (_bShowParticles)
    {
        _particles.update();
    }
}

//--------------------------------------------------------------
void ofApp::draw() {

    ofClear(0, 0, 0, 255);

    if (_bReceiveOSC) {
        _osc.update();
    }

    if (_texDepth.size() > 0 && _bTexturesInitialized) {
        if (_bShowGui) {
            _texRGB[0].draw(0, 0, ofGetWidth(), ofGetHeight());
        }

        if (_bShowMoments) {
            ofPushStyle();
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_LINE_SMOOTH);
            glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);
            glCallList(_displayList);
            glDisable(GL_LINE_SMOOTH);
            glDisable(GL_BLEND);
            ofPopStyle();
            ofPushStyle();
            ofPushMatrix();
            ofTranslate(ofGetWidth() * .5f, ofGetHeight() * .5f);
            if (_bFlipParticles) {
                ofSetColor(255, 0, 0);
            } else {
                ofSetColor(0, 255, 0);
            }
            ofDrawCircle(ofPoint(_momentCenter.x, _momentCenter.y), 10);
            ofPopMatrix();
            ofPopStyle();
        }

        if (_bShowParticles) {
            ofPushStyle();
            ofPushMatrix();
            ofTranslate(ofGetWidth() * .5f, ofGetHeight() * .5f);
            ofEnableBlendMode(OF_BLENDMODE_ADD);
            _particles.draw();
            ofDisableBlendMode();
            ofPopMatrix();
            ofPopStyle();
        }

        if (_bShowGui) {
            ofPushStyle();
            _texDepth[0].draw(ofGetWidth() - 20 - _texDepth[0].getWidth(), 20);
            _depthImage.draw(ofGetWidth() - 20 - _depthImage.getWidth() - 20 - _texDepth[0].getWidth(),
                    20, _texDepth[0].getWidth(), _texDepth[0].getHeight());

            ofDrawBitmapString(ofToString(ofGetFrameRate(), 2, 2), ofGetWidth() - 200, ofGetHeight() - 80);
            ofDrawBitmapString(ofToString(_momentCenter.x, 2, 4) + ' ' + ofToString(_momentCenter.y, 2, 4),
                    ofGetWidth() - 200, ofGetHeight() - 40);
            ofPopStyle();

#ifdef USE_OPTICAL_FLOW
            drawFlowColored(_texDepth[0].getWidth(), _texDepth[0].getHeight());
#endif

            _panel.draw();
        }
    }
}

//--------------------------------------------------------------
void ofApp::particleSetup() {
    uint16_t w = (uint16_t) ofGetWidth();
    uint16_t h = (uint16_t) ofGetHeight();

    _particles.init(w, h);

    if (ofIsGLProgrammableRenderer()) {
        ofLogNotice() << "Using GL programmable renderer.";
        _particles.loadShaders("shaders330/update", "shaders330/draw");
    } else {
        _particles.loadShaders("shaders120/update", "shaders120/draw");
    }

    ofLogNotice() << "Adding particle count: " << w * h;

    // use new to allocate floats on the heap rather than the stack
    float* positions = new float[w * h * 4];
    for (uint16_t y = 0; y < h; ++y)
    {
        for (uint16_t x = 0; x < w; ++x)
        {
            uint16_t idx = y * w + x;
            positions[idx * 4] = (x - ofGetWidth() * .5f) * .25f;
            positions[idx * 4 + 1] = (y - ofGetHeight() * .5f) * .5f;
            positions[idx * 4 + 2] = 0.f;
            positions[idx * 4 + 3] = 0.f;
        }
    }

    _particles.loadDataTexture(ofxGpuParticles::POSITION, positions);
    delete[] positions;

    _particles.zeroDataTexture(ofxGpuParticles::VELOCITY);
}

#ifdef USE_OPTICAL_FLOW
void ofApp::drawFlowColored(float width, float height, float degMin, float degMax, float velMin, float velMax) {
    ofPoint vel;
    ofMesh velMesh;
    float angle;

    velMesh.setMode(OF_PRIMITIVE_LINES);

    for(int x=0; x<width; x+=3) {
        for(int y=0; y<height; y+=3) {
            vel = flowSolver.getVelAtPixel(x, y);
            ofVec3f p(x,y);
            ofVec3f vc = vel;
            vc.normalize();
            angle = atan2(vc.y, vc.x)+3.14159265f;
            if(vel.length() < velMin || vel.length() >  velMax
                    || angle < ofDegToRad(degMin)
                    || angle > ofDegToRad(degMax)) {
                continue;
            }
            float hue = angle/(3.14159265f*2.0f);
            ofFloatColor c;
            c.setHsb(hue, 1.f, 1.f);
            c.a = 0.25f;
            velMesh.addColor(c);
            velMesh.addVertex(p);
            c.a = 0.f;
            velMesh.addColor(c);
            velMesh.addVertex(p+vel*lineScale);
        }
    }

    ofPushStyle();
    ofEnableAlphaBlending();
    velMesh.draw();
    ofPopStyle();
}
#endif

//--------------------------------------------------------------
void ofApp::exit() {
    _panel.saveToFile("settings.xml");
    for (uint8_t i = 0; i < _kinect.size(); i +=1) {
        _kinect[i].get()->close();
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    switch (key) {
        case 'f':
            ofToggleFullscreen();
            break;
        case 'p':
            _bShowParticles = !_bShowParticles;
            if (_bShowParticles) {
                particleSetup();
            }
            break;
        case 'm':
            _bShowMoments = !_bShowMoments;
            break;
        case 'g':
            _bShowGui = !_bShowGui;
            break;
        case 'y':
            _bFlipImage = !_bFlipImage;
            break;
        case 'x':
            _bFlipParticles = !_bFlipParticles;
            break;
        default:
            break;
    }
}