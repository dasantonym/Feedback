#include "ofApp.h"

using namespace cv;
using namespace ofxCv;

//--------------------------------------------------------------
void ofApp::setup() {
    ofSetLogLevel(OF_LOG_NOTICE);

    lineScale = 10.f;
    velocityMax = 1000.f;
    velocityMin = 2.f;
    angleMin = 0.f;
    angleMax = 360.f;

    amp = 1.f;
    attack = 1.f;

    bShowParticles = false;
    bShowMoments = false;
    bShowGui = false;
    bUseOpticalFlow = false;

    bFlipImage = true;
    bFlipParticles = true;

    receiver.setup(5556);

    panel.setup("", "settings.xml", 10, 100);

    ofxKinectV2 tmp;
    vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();
    kinects.resize(deviceList.size());
    texDepth.resize(kinects.size());
    texRGB.resize(kinects.size());

    for(uint8_t d = 0; d < kinects.size(); d++){
        kinects[d] = shared_ptr <ofxKinectV2> (new ofxKinectV2());
        kinects[d]->open(deviceList[d].serial);
        panel.add(kinects[d]->params);
    }

    panel.loadFromFile("settings.xml");

    outImage.setImageType(OF_IMAGE_GRAYSCALE);

    displayList = glGenLists(1);

    particleSetup();

    ofAddListener(particles.updateEvent, this, &ofApp::onParticlesUpdate);
}

void ofApp::onParticlesUpdate(ofShader& shader)
{
    shader.setUniform3fv("mouse", _mouse.getPtr());
    shader.setUniform1f("elapsed", (float)ofGetLastFrameTime());
    shader.setUniform1f("radiusSquared", 600.f * 600.f);
}

//--------------------------------------------------------------
void ofApp::update() {

    ofBackground(100, 100, 100);

    while (receiver.hasWaitingMessages())
    {
        ofxOscMessage m;
        receiver.getNextMessage(m);

        if (m.getAddress() == "/bonk")
        {
            float val = m.getArgAsFloat(1) / 100.f;
            if (val > 1.f)
            {
                val = 1.f;
            }
            attack = val;
        } else {
            attack *= .5f;
        }

        if (m.getAddress() == "/pitch")
        {
            float val = m.getArgAsFloat(0)/80.f;
            if (val > 1.f)
            {
                val = 1.f;
            }
            pitch = val;
        } else {
            pitch *= .5f;
        }

        if (m.getAddress() == "/amp")
        {
            float val = m.getArgAsFloat(0)/100.f;

            if (val > 1.f)
            {
                val = 1.f;
            }
            amp = val;
        }

    }

    for(int d = 0; d < kinects.size(); d++){
        kinects[d]->update();
        if( kinects[d]->isFrameNew() ){
            texDepth[d].loadData( kinects[d]->getDepthPixels() );
            texRGB[d].loadData( kinects[d]->getRgbPixels() );

            if (d == 0 && !bTexturesInitialized) {
                uint16_t width = (uint16_t)texDepth[0].getWidth();
                uint16_t height = (uint16_t)texDepth[0].getHeight();

                flowSolver.setup(width, height, .35f, 5, 10, 1, 3, 2.25f, false, false);

                outImage.allocate(width, height, OF_IMAGE_GRAYSCALE);
                grayImage = Mat::zeros(height, width, CV_8UC1);
                grayImageAvg = Mat::zeros(height, width, CV_32FC1);

                bTexturesInitialized = true;
            }

            if (d == 0) {
                ofPixels depthPixels;
                texDepth[d].readToPixels(depthPixels);
                outImage.setFromPixels(depthPixels);
                grayImage = toCv(outImage);

                if (bFlipImage) cv::flip(grayImage, grayImage, 1);

                blur(grayImage, grayImage, (uint8_t)roundf(12.f*amp+1.f));

                cv::accumulateWeighted(grayImage, grayImageAvg, .2f);
                grayImageAvg.convertTo(grayImage, CV_8UC1);

                outImage.update();

                float scale = ofGetWidth() / outImage.getWidth();
                int16_t yoffset = (int16_t)roundf(((outImage.getHeight() * scale) - ofGetHeight()) * .5f);

                cv::findContours(grayImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);

                int16_t largeIndex = -1;
                double largeSize = 0.;

                for (uint16_t i = 0; i < contours.size(); i++)
                {
                    double areaSize = contourArea(contours[i]);
                    if (areaSize > largeSize)
                    {
                        largeSize = areaSize;
                        largeIndex = i;
                    }
                }

                if (largeIndex > 0)
                {
                    Moments largeMoments = moments(contours[largeIndex]);
                    mc = Point2f( (float)(largeMoments.m10/largeMoments.m00), (float)(largeMoments.m01/largeMoments.m00) );
                    _mouse.x = mc.x * scale - .5f * ofGetWidth();
                    _mouse.y = mc.y * scale - .5f * ofGetHeight() - yoffset;

                    if (bFlipParticles) _mouse.x *= -1.f;

                    if (bShowMoments) {
                        glNewList(displayList, GL_COMPILE);
                        glColor4f(1.f, 1.f, 1.f, .5f);

                        glBegin(GL_LINE_STRIP);
                        for (uint16_t cn = 0; cn < contours[largeIndex].size(); cn++) {
                            glVertex3d(float(contours[largeIndex][cn].x) * scale, float(contours[largeIndex][cn].y) * scale - yoffset, 0.f);
                        }
                        glVertex3d(float(contours[largeIndex][0].x) * scale, float(contours[largeIndex][0].y) * scale - yoffset, 0.f);
                        glEnd();

                        glEndList();
                    }
                }
                if (bUseOpticalFlow) {
                    flowSolver.update(outImage);
                }
            }
        }
    }

    if (bShowParticles)
    {
        particles.update();
    }
}

//--------------------------------------------------------------
void ofApp::draw() {

    ofClear(0, 0, 0, 255);

    if (texDepth.size() > 0 && bTexturesInitialized) {
        if (bShowMoments) {
            ofPushStyle();
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_LINE_SMOOTH);
            glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);
            glCallList(displayList);
            glDisable(GL_LINE_SMOOTH);
            glDisable(GL_BLEND);
            ofPopStyle();
            ofPushStyle();
            ofPushMatrix();
            ofTranslate(ofGetWidth() * .5f, ofGetHeight() * .5f);
            if (bFlipParticles) {
                ofSetColor(255, 0, 0);
            } else {
                ofSetColor(0, 255, 0);
            }
            ofDrawCircle(ofPoint(_mouse.x, _mouse.y), 10);
            ofPopMatrix();
            ofPopStyle();
        }

        if (bShowParticles) {
            ofPushStyle();
            ofPushMatrix();
            ofTranslate(ofGetWidth() * .5f, ofGetHeight() * .5f);
            ofEnableBlendMode(OF_BLENDMODE_ADD);
            particles.draw();
            ofDisableBlendMode();
            ofPopMatrix();
            ofPopStyle();
        }

        if (bShowGui) {
            ofPushStyle();
            texDepth[0].draw(ofGetWidth() - 20 - texDepth[0].getWidth(), 20);
            texRGB[0].draw(ofGetWidth() - 20 - texRGB[0].getWidth() * .25f,
                    texDepth[0].getHeight() + 40, texRGB[0].getWidth() * .25f, texRGB[0].getHeight() * .25f);
            outImage.draw(ofGetWidth() - 20 - outImage.getWidth() - 20 - texDepth[0].getWidth(),
                    20, texDepth[0].getWidth(), texDepth[0].getHeight());

            ofDrawBitmapString(ofToString(ofGetFrameRate(), 2, 2), ofGetWidth() - 200, ofGetHeight() - 80);
            ofDrawBitmapString(ofToString(_mouse.x, 2, 4) + ' ' + ofToString(_mouse.y, 2, 4),
                    ofGetWidth() - 200, ofGetHeight() - 40);
            ofPopStyle();

            if (bUseOpticalFlow) drawFlowColored(texDepth[0].getWidth(), texDepth[0].getHeight());

            panel.draw();
        }
    }
}

void ofApp::drawFlowColored(float width, float height) {
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
            if(vel.length() < velocityMin || vel.length() >  velocityMax
                    || angle < ofDegToRad(angleMin)
                    || angle > ofDegToRad(angleMax)) {  // outside limits, no point drawing.
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

//--------------------------------------------------------------
void ofApp::particleSetup() {
    uint16_t w = (uint16_t) ofGetWidth();
    uint16_t h = (uint16_t) ofGetHeight();

    particles.init(w, h);
    if (ofIsGLProgrammableRenderer()) {
        ofLogNotice() << "Using GL programmable renderer.";
        particles.loadShaders("shaders330/update", "shaders330/draw");
    } else {
        particles.loadShaders("shaders120/update", "shaders120/draw");
    }

    ofLogNotice() << "Adding particle count: " << w * h;

    // use new to allocate floats on the heap rather than the stack
    float* particlePos = new float[w * h * 4];
    for (uint16_t y = 0; y < h; ++y)
    {
        for (uint16_t x = 0; x < w; ++x)
        {
            uint16_t idx = y * w + x;
            particlePos[idx * 4] = (x - w * .5f);
            particlePos[idx * 4 + 1] = (y - h * .5f);
            particlePos[idx * 4 + 2] = 0.f;
            particlePos[idx * 4 + 3] = 0.f;
        }
    }
    particles.zeroDataTexture(ofxGpuParticles::VELOCITY);
    particles.loadDataTexture(ofxGpuParticles::POSITION, particlePos);
    delete[] particlePos;
}

//--------------------------------------------------------------
void ofApp::exit() {
    panel.saveToFile("settings.xml");
    for (uint8_t i = 0; i < kinects.size(); i +=1) {
        kinects[i].get()->close();
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    switch (key) {
        case 'f':
            ofToggleFullscreen();
            break;
        case 'p':
            bShowParticles = !bShowParticles;
            if (bShowParticles) {
                particleSetup();
            }
            break;
        case 'm':
            bShowMoments = !bShowMoments;
            break;
        case 'g':
            bShowGui = !bShowGui;
            break;
        case 'y':
            bFlipImage = !bFlipImage;
            break;
        case 'x':
            bFlipParticles = !bFlipParticles;
            break;
        default:
            break;
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}