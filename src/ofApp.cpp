#include "ofApp.h"
#include "DepthAnalysis.h"

//--------------------------------------------------------------
void ofApp::setup() {
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetVerticalSync(true);

    _panel.setup("", "settings.xml", 20, 20);

    _panel.add(_optsShowGui.setup("showGui", false));
    _panel.add(_optsShowStats.setup("showStats", false));
    _panel.add(_optsShowMoments.setup("showMoments", false));
    _panel.add(_optsReceiveOSC.setup("receiveOSC", false));
    _panel.add(&_depthAnalysis.optsFlipImage);
    _panel.add(&_depthAnalysis.optsFlipCenter);
    _panel.add(_optsFreezeParticles.setup("freezeParticles", false));

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

    _displayList = glGenLists(1);

    _osc.setup();

    _viewSize.x = ofGetWidth();
    _viewSize.y = ofGetHeight();

    _panel.loadFromFile("settings.xml");
}

void ofApp::onParticlesUpdate(ofShader& shader)
{
    shader.setUniform3fv("momentCenter", _moments.center.getPtr());
    shader.setUniform2fv("viewSize", _viewSize.getPtr());
    shader.setUniform1f("elapsed", (float) ofGetLastFrameTime());
    shader.setUniform1f("radiusSquared", (float) pow(ofGetHeight(), 2.f));
}

//--------------------------------------------------------------
void ofApp::update() {
    if (_optsReceiveOSC) {
        _osc.update();
    }

    for(uint8_t d = 0; d < _kinect.size(); d++){
        if (d != 0) {
            continue;
        }
        _kinect[d]->update();
        if(_kinect[d]->isFrameNew()) {
            _texDepth[d].loadData(_kinect[d]->getDepthPixels());
            _texRGB[d].loadData(_kinect[d]->getRgbPixels());

            if (!_bTexturesInitialized) {
                uint16_t depthWidth = (uint16_t) _texDepth[0].getWidth();
                uint16_t depthHeight = (uint16_t) _texDepth[0].getHeight();

                _depthAnalysis.init(depthWidth, depthHeight);
                _analysisConfig = _depthAnalysis.getConfig();

#ifdef USE_OPTICAL_FLOW
                flowSolver.setup(depthWidth, depthHeight, .35f, 5, 10, 1, 3, 2.25f, false, false);
#endif

                _bTexturesInitialized = true;
            }
        }

        if (!_bTexturesInitialized) {
            continue;
        }

        _depthAnalysis.update(&_texDepth[d]);

        if (_depthAnalysis.isUpdated()) {
            _moments = _depthAnalysis.getMoments();

            if (_optsShowMoments) {
                uint16_t idx = (uint16_t) _moments.index;
                float scale = ofGetWidth() / _texDepth[d].getWidth();

                glNewList(_displayList, GL_COMPILE);
                glColor4f(1.f, 1.f, 1.f, .5f);

                glBegin(GL_LINE_STRIP);
                for (uint16_t cn = 0; cn < _moments.contour.size(); cn++) {
                    glVertex3d(float(_moments.contour[cn].x) * scale,
                            float(_moments.contour[cn].y) * scale - _analysisConfig.offset.y, 0.f);
                }
                glVertex3d(float(_moments.contour[0].x) * scale,
                        float(_moments.contour[0].y) * scale - _analysisConfig.offset.y, 0.f);
                glEnd();

                glEndList();
            }
        }
    }

    if (_bTexturesInitialized) {
#ifdef USE_OPTICAL_FLOW
        if (_bTexturesInitialized) {
            flowSolver.update(_depthImage);
        }
#endif

        if (_bParticlesInitialized && !_optsFreezeParticles) {
            _particles.update();
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw() {
    ofClear(0, 0, 0, 255);

    if (_kinect.size() > 0 && _bTexturesInitialized) {
        if (_optsShowGui) {
            _texRGB[0].draw(0, 0, ofGetWidth(), ofGetHeight());
        }

        if (_optsShowMoments) {
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
            if (_depthAnalysis.optsFlipCenter) {
                ofSetColor(255, 0, 0);
            } else {
                ofSetColor(0, 255, 0);
            }
            ofDrawCircle(ofPoint(_moments.center.x, _moments.center.y), 10);
            ofPopMatrix();
            ofPopStyle();
        }

        if (_bParticlesInitialized) {
            ofPushMatrix();
            ofTranslate(ofGetWidth() * .5f, ofGetHeight() * .5f);
            ofEnableBlendMode(OF_BLENDMODE_ADD);
            _particles.draw();
            ofDisableBlendMode();
            ofPopMatrix();
        }

        if (_optsShowGui) {
            _texDepth[0].draw(ofGetWidth() - 20 - _texDepth[0].getWidth(), 20);

#ifdef USE_OPTICAL_FLOW
            drawFlowColored(_texDepth[0].getWidth(), _texDepth[0].getHeight());
#endif

            _panel.draw();
        }

        if (_optsShowStats) {
            ofDrawBitmapString(ofToString(ofGetFrameRate(), 2, 2, '0'), ofGetWidth() - 200, ofGetHeight() - 80);
            ofDrawBitmapString(ofToString(_moments.center.x, 2, 3, '0') + ' ' + ofToString(_moments.center.y, 2, 3, '0'),
                    ofGetWidth() - 200, ofGetHeight() - 40);
        }
    }
}

//--------------------------------------------------------------
void ofApp::particleSetup() {
    uint16_t w = PARTICLE_GRID_X;
    uint16_t h = PARTICLE_GRID_Y;

    _particles.init(w, h, OF_PRIMITIVE_POINTS, false);

    if (!_bParticlesInitialized) {
        ofLogNotice() << "Loading shaders...";
        if (ofIsGLProgrammableRenderer()) {
            ofLogNotice() << "Using GL programmable renderer.";
            _particles.loadShaders("shaders330/update", "shaders330/draw");
        } else {
            _particles.loadShaders("shaders120/update", "shaders120/draw");
        }

        ofAddListener(_particles.updateEvent, this, &ofApp::onParticlesUpdate);

        _bParticlesInitialized = true;
    }

    ofLogNotice() << "Adding particle count: " << w * h;

    // use new to allocate floats on the heap rather than the stack
    float* positions = new float[w * h * 4];
    for (uint16_t y = 0; y < h; ++y)
    {
        for (uint16_t x = 0; x < w; ++x)
        {
            uint32_t idx = y * w + x;
            positions[idx * 4] = (x - w * .5f) * .25f;
            positions[idx * 4 + 1] = (y - h * .5f) * -.25f;
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
            velMesh.addVertex(p+vel*10.f);
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
        case ' ':
            particleSetup();
            break;
        case 'm':
            _optsShowMoments = !_optsShowMoments;
            break;
        case 'g':
            _optsShowGui = !_optsShowGui;
            break;
        case 's':
            _optsShowStats = !_optsShowStats;
            break;
        case 'y':
            _depthAnalysis.optsFlipImage = !_depthAnalysis.optsFlipImage;
            break;
        case 'x':
            _depthAnalysis.optsFlipCenter = !_depthAnalysis.optsFlipCenter;
            break;
        default:
            break;
    }
}

void ofApp::windowResized(int w, int h) {
    _viewSize.x = w;
    _viewSize.y = h;
}