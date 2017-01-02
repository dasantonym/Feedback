#include "ofApp.h"

using namespace cv;
using namespace ofxCv;

//--------------------------------------------------------------
void ofApp::setup() {
    ofSetLogLevel(OF_LOG_VERBOSE);

    lineScale = 10.0;
    velocityMax = 1000.0;
    velocityMin = 2.0;
    angleMin = 0.0;
    angleMax = 360.0;

    amp = 0.0;
    attack = 0.0;

    drawCaustics = true;
    drawParticles = false;
    drawOutlines = false;

    flipImage = true;
    flipParticles = true;

    receiver.setup(5556);

    panel.setup("", "settings.xml", 10, 100);

    ofxKinectV2 tmp;
    vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();
    kinects.resize(deviceList.size());
    texDepth.resize(kinects.size());
    texRGB.resize(kinects.size());

    for(int d = 0; d < kinects.size(); d++){
        kinects[d] = shared_ptr <ofxKinectV2> (new ofxKinectV2());
        kinects[d]->open(deviceList[d].serial);
        panel.add(kinects[d]->params);
    }

    outImage.setImageType(OF_IMAGE_GRAYSCALE);

    if (kinects.size() > 0) {
        uint16_t width = (uint16_t)texDepth[0].getWidth();
        uint16_t height = (uint16_t)texDepth[0].getHeight();

        caustics.setup(ofGetWidth(), ofGetHeight(), 1000, ofGetWidth()/width);
        flowSolver.setup(width, height, 0.35, 5, 10, 1, 3, 2.25, false, false);

        inImage.allocate(width, height, OF_IMAGE_GRAYSCALE);
        outImage.allocate(width, height, OF_IMAGE_GRAYSCALE);
        grayImage = Mat::zeros(height, width, CV_8UC1);
        grayImageAvg = Mat::zeros(height, width, CV_32FC1);

        grayThreshNear = Mat::zeros(height, width, CV_8UC1);
        grayThreshFar = Mat::zeros(height, width, CV_8UC1);
    }

    displayList = glGenLists(1);

    nearThreshold = 50;
    farThreshold = 9;
    bThreshWithOpenCV = true;

    // ofSetFrameRate(60);

    // zero the tilt on startup
    angle = 9;

    // 1,000,000 particles
    unsigned w = 500;
    unsigned h = 500;

    particles.init(w, h);

    // initial positions
    // use new to allocate 4,000,000 floats on the heap rather than
    // the stack
    float* particlePosns = new float[w * h * 4];
    for (unsigned y = 0; y < h; ++y)
    {
        for (unsigned x = 0; x < w; ++x)
        {
            unsigned idx = y * w + x;
            particlePosns[idx * 4] = 400.f * x / (float)w - 200.f; // particle x
            particlePosns[idx * 4 + 1] = 400.f * y / (float)h - 200.f; // particle y
            particlePosns[idx * 4 + 2] = 0.f; // particle z
            particlePosns[idx * 4 + 3] = 0.f; // dummy
        }
    }
    particles.loadDataTexture(ofxGpuParticles::POSITION, particlePosns);
    delete[] particlePosns;

    // initial velocities
    particles.zeroDataTexture(ofxGpuParticles::VELOCITY);

    // listen for update event to set additonal update uniforms
    ofAddListener(particles.updateEvent, this, &ofApp::onParticlesUpdate);
    ofAddListener(particles.drawEvent, this, &ofApp::onParticlesDraw);

    // start from the front
    bDrawPointCloud = false;
}

void ofApp::onParticlesUpdate(ofShader& shader)
{
    float flip = 1.0f;
    if (flipParticles) {
        flip = -1.0f;
    }
    //ofVec3f mouse(ofGetMouseX() - .5f * ofGetWidth(), .5f * ofGetHeight() - ofGetMouseY() , 0.f);
    ofVec3f mouse((mc.x - .5f * ofGetWidth()) * flip, .5f * ofGetHeight() - mc.y, 0.f);
    // ofVec3f ext1(mc.x - .5f * ofGetWidth(), .5f * ofGetHeight() - mc.y , 0.f);
    shader.setUniform3fv("mouse", mouse.getPtr());
    shader.setUniform1f("elapsed", (float)ofGetLastFrameTime());
    shader.setUniform1f("radiusSquared", 300.f * 300.f);
}

void ofApp::onParticlesDraw(ofShader& shader)
{
    //ofVec3f mouse(ofGetMouseX() - .5f * ofGetWidth(), .5f * ofGetHeight() - ofGetMouseY() , 0.f);
    //ofVec3f mouse(mc.x - .5f * ofGetWidth(), .5f * ofGetHeight() - mc.y , 0.f);
    //shader.setUniform3fv("mouse", mouse.getPtr());
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
            if (val > 1.0)
            {
                val = 1.0;
            }
            attack = val;
        } else {
            attack *= 0.5;
        }

        if (m.getAddress() == "/pitch")
        {
            float val = m.getArgAsFloat(0)/80.f;
            if (val > 1.0)
            {
                val = 1.0;
            }
            pitch = val;
        } else {
            pitch *= 0.5;
        }

        if (m.getAddress() == "/amp")
        {
            float val = m.getArgAsFloat(0)/100.f;

            if (val > 1.0)
            {
                val = 1.0;
            }
            amp = val;
        }

    }

    for(int d = 0; d < kinects.size(); d++){
        kinects[d]->update();
        if( kinects[d]->isFrameNew() ){
            texDepth[d].loadData( kinects[d]->getDepthPixels() );
            texRGB[d].loadData( kinects[d]->getRgbPixels() );

            if (d == 0) {
                ofPixels pixels;
                texDepth[d].readToPixels(pixels);
                outImage.setFromPixels(pixels);
                outImage.resize(ofGetWidth(), ofGetHeight());
                grayImage = toCv(outImage);

                if (flipImage)
                {
                    cv::flip(grayImage, grayImage, 1);
                }

                blur(grayImage, grayImage, (uint8_t)roundf(12.0f*amp+1.0f));

                cv::accumulateWeighted(grayImage, grayImageAvg, 0.2);

                Mat tmp;
                grayImageAvg.convertTo(tmp, CV_8UC1);

                threshold(tmp, grayThreshNear, nearThreshold, 255, CV_THRESH_BINARY_INV);
                threshold(tmp, grayThreshFar, farThreshold, 255, CV_THRESH_BINARY);

                bitwise_and(grayThreshNear, grayThreshFar, tmp);

                toOf(tmp, outImage);
                outImage.update();

                cv::findContours(tmp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);

                glNewList(displayList, GL_COMPILE);
                glColor4f(1.0, 1.0, 1.0, 0.5);

                float scaleFactorX = float(ofGetWidth())/texDepth[d].getWidth();
                float scaleFactorY = float(ofGetHeight())/texDepth[d].getHeight();

                double largeIndex = -1.0;
                double largeSize = 0.0;

                for (unsigned int i = 0; i < contours.size(); i++)
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
                    mc = Point2f( (float)(largeMoments.m10/largeMoments.m00) * scaleFactorX , (float)(largeMoments.m01/largeMoments.m00) * scaleFactorY );

                    if (drawCaustics)
                    {
                        caustics.reset();
                    }

                    glBegin(GL_LINE_STRIP);
                    for (unsigned int cn = 0; cn < contours[largeIndex].size(); cn++)
                    {
                        if (drawCaustics)
                        {
                            caustics.addPoint(ofVec2f(contours[largeIndex][cn].x, contours[largeIndex][cn].y));
                        }
                        glVertex3d(float(contours[largeIndex][cn].x)*scaleFactorX, float(contours[largeIndex][cn].y)*scaleFactorY, 0.0);
                    }
                    glVertex3d(float(contours[largeIndex][0].x)*scaleFactorX, float(contours[largeIndex][0].y)*scaleFactorY, 0.0);
                    glEnd();

                    // add caustics points on the borders

                    if (drawCaustics)
                    {

                        float factor = pitch * 15 + 1;
                        for (uint8_t x = 0; x < ofGetWidth(); x += ofGetWidth() / factor)
                        {
                            caustics.addPoint(ofVec2f(x, 0.0));
                            caustics.addPoint(ofVec2f(x, ofGetHeight()));
                        }

                        for (uint8_t y = 0; y < ofGetHeight(); y += ofGetHeight() / factor)
                        {
                            caustics.addPoint(ofVec2f(0.0, y));
                            caustics.addPoint(ofVec2f(ofGetWidth(), y));
                        }

                        // this triggers the delaunay
                        caustics.update(amp);
                    }
                }
                glEndList();

                //flowSolver.update(outImage);
            }
        }
    }

    if (drawParticles)
    {
        particles.update();
    }
}

//--------------------------------------------------------------
void ofApp::draw() {

    ofClear(0, 0, 0, 255);

    //outImage.draw(0, 0, ofGetWidth(), ofGetHeight());
    //contourFinder.draw(10, 320, 400, 300);

    if (texDepth.size() > 0) {
        // drawFlowColored(texDepth[0].getWidth(), texDepth[0].getHeight());

        if (drawCaustics) {
            caustics.renderToFbo();
            caustics.getFbo().draw(0, 0, ofGetWidth(), ofGetHeight());
        }

        if (drawOutlines) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_LINE_SMOOTH);
            glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);
            glCallList(displayList);
            glDisable(GL_LINE_SMOOTH);
            glDisable(GL_BLEND);
        }

        if (drawParticles) {
            easyCam.begin();
            ofEnableBlendMode(OF_BLENDMODE_ADD);
            particles.draw();
            ofDisableBlendMode();
            easyCam.end();
        }
    }

    panel.draw();

    // draw instructions
    /*
    ofSetColor(255, 255, 255);
    stringstream reportStream;

    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
        << "motor / led / accel controls are not currently supported" << endl << endl;
    }

    reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
    << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
    << "set near threshold " << nearThreshold << " (press: + -)" << endl
    << "set far threshold " << farThreshold << " (press: < >) num blobs found "// << contourFinder.nBlobs
    << ", fps: " << ofGetFrameRate() << endl
    << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
     */

    //ofDrawBitmapString(reportStream.str(), 20, 652);



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
            c.setHsb(hue, 1.0, 1.0);
            c.a = 0.25;
            velMesh.addColor(c);
            velMesh.addVertex(p);
            c.a = 0.0;
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
void ofApp::exit() {
    for (uint8_t i = 0; i < kinects.size(); i +=1) {
        kinects[i].get()->close();
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    switch (key) {
        case 'f':
            ofSetFullscreen(true);
        case ' ':
            bThreshWithOpenCV = !bThreshWithOpenCV;
            break;

        case'p':
            bDrawPointCloud = !bDrawPointCloud;
            break;

        case 'a':
            drawCaustics = !drawCaustics;
            break;

        case 's':
            if (!drawParticles)
            {
                unsigned w = 500;
                unsigned h = 500;

                particles.init(w, h);
                // initial positions
                // use new to allocate 4,000,000 floats on the heap rather than
                // the stack
                float* particlePosns = new float[w * h * 4];
                for (unsigned y = 0; y < h; ++y)
                {
                    for (unsigned x = 0; x < w; ++x)
                    {
                        unsigned idx = y * w + x;
                        particlePosns[idx * 4] = 400.f * x / (float)w - 200.f; // particle x
                        particlePosns[idx * 4 + 1] = 400.f * y / (float)h - 200.f; // particle y
                        particlePosns[idx * 4 + 2] = 0.f; // particle z
                        particlePosns[idx * 4 + 3] = 0.f; // dummy
                    }
                }
                particles.loadDataTexture(ofxGpuParticles::POSITION, particlePosns);
                delete[] particlePosns;

                // initial velocities
                particles.zeroDataTexture(ofxGpuParticles::VELOCITY);
            }
            drawParticles = !drawParticles;
            break;

        case 'd':
            drawOutlines = !drawOutlines;
            break;

        case 'y':
            flipImage = !flipImage;
            break;

        case 'x':
            flipParticles = !flipParticles;
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