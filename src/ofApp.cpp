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
    
    if (USE_KINECT2)
    {
        kinect2.open();
        outImage.setImageType(OF_IMAGE_GRAYSCALE);
    }
    else
    {
        // enable depth->video image calibration
        kinect.setRegistration(true);
        
        kinect.init();
        //kinect.init(true); // shows infrared instead of RGB video image
        //kinect.init(false, false); // disable video image (faster fps)
        
        kinect.open();		// opens first available kinect
        //kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
        //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
        
        kinect.setDepthClipping(750);
        
        // print the intrinsic IR sensor values
        if(kinect.isConnected()) {
            ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
            ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
            ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
            ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
        }
    }
    
    caustics.setup(ofGetWidth(), ofGetHeight(), 1000, ofGetWidth()/kinect.width);
    
    displayList = glGenLists(1);
    
    flowSolver.setup(kinect.width, kinect.height, 0.35, 5, 10, 1, 3, 2.25, false, false);
    
    inImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    outImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayImage = Mat::zeros(400, kinect.width, CV_8UC1);
    grayImageAvg = Mat::zeros(400, kinect.width, CV_32FC1);
    
    grayThreshNear = Mat::zeros(400, kinect.width, CV_8UC1);
    grayThreshFar = Mat::zeros(400, kinect.width, CV_8UC1);
    
    nearThreshold = 50;
    farThreshold = 9;
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 9;
    kinect.setCameraTiltAngle(angle);
    
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
    float flip = 1.0;
    if (flipParticles) {
        flip = -1.0;
    }
    //ofVec3f mouse(ofGetMouseX() - .5f * ofGetWidth(), .5f * ofGetHeight() - ofGetMouseY() , 0.f);
    ofVec3f mouse((mc.x - .5f * ofGetWidth()) * flip, .5f * ofGetHeight() - mc.y, 0.f);
    ofVec3f ext1(mc.x - .5f * ofGetWidth(), .5f * ofGetHeight() - mc.y , 0.f);
    shader.setUniform3fv("mouse", mouse.getPtr());
    shader.setUniform1f("elapsed", ofGetLastFrameTime());
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
        receiver.getNextMessage(&m);
        
        if (m.getAddress() == "/bonk")
        {
            float val = m.getArgAsFloat(1)/100.0;
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
            float val = m.getArgAsFloat(0)/80.0;
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
            float val = m.getArgAsFloat(0)/100.0;
            
            if (val > 1.0)
            {
                val = 1.0;
            }
            amp = val;
        }

    }
    
    bool frameNew;
    
    if (USE_KINECT2)
    {
        kinect2.update();
        frameNew = kinect2.isFrameNew();
    }
    else
    {
        kinect.update();
        frameNew = kinect.isFrameNew();
    }
    
    if(frameNew) {
        
        if (USE_KINECT2)
        {
            texDepth.loadData( kinect2.getDepthPixels() );
            outImage.setFromPixels(kinect2.getDepthPixels());
            outImage.resize(ofGetWidth(), ofGetHeight());
            grayImage = toCv(outImage);
        }
        else
        {
            outImage.setFromPixels(kinect.getDepthPixelsRef());
            cv::Rect myROI(0, 40, 640, 400);
            grayImage = toCv(outImage)(myROI);
        }
        
        if (flipImage)
        {
            cv::flip(grayImage, grayImage, 1);
        }
        
        blur(grayImage, grayImage, 12.0*amp+1.0);
        
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
        
        float scaleFactorX = float(ofGetWidth())/kinect.width;
        float scaleFactorY = float(ofGetHeight())/400;
        
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
            mc = Point2f( (largeMoments.m10/largeMoments.m00) * scaleFactorX , (largeMoments.m01/largeMoments.m00) * scaleFactorY );
            
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
                
            unsigned int factor = pitch * 15 + 1;
            for (unsigned int x = 0; x < ofGetWidth(); x += ofGetWidth() / factor)
            {
                caustics.addPoint(ofVec2f(x, 0.0));
                caustics.addPoint(ofVec2f(x, ofGetHeight()));
            }
            
            for (unsigned int y = 0; y < ofGetHeight(); y += ofGetHeight() / factor)
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
    
    if (drawParticles)
    {
    particles.update();
    }
}

//--------------------------------------------------------------
void ofApp::draw() {
    
    ofClear(0, 0, 0, 255);
    
    if(bDrawPointCloud) {
        easyCam.begin();
        drawPointCloud();
        easyCam.end();
    } else {
        // draw from the live kinect
        //kinect.drawDepth(10, 10, 400, 300);
        //kinect.draw(420, 10, 400, 300);
        
        //outImage.draw(0, 0, ofGetWidth(), ofGetHeight());
        //contourFinder.draw(10, 320, 400, 300);
        //drawFlowColored();
        
        if (drawCaustics)
        {
        caustics.renderToFbo();
        caustics.getFbo().draw(0, 0, ofGetWidth(), ofGetHeight());
        }
        
        if (drawOutlines)
        {
        glEnable( GL_BLEND );
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable( GL_LINE_SMOOTH );
        glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);
        glCallList(displayList);
        glDisable( GL_LINE_SMOOTH );
        glDisable( GL_BLEND );
        }
        
        if (drawParticles)
        {
        easyCam.begin();
        ofEnableBlendMode(OF_BLENDMODE_ADD);
        particles.draw();
        ofDisableBlendMode();
        easyCam.end();
        }
        
    }
    
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

void ofApp::drawFlowColored() {
    ofPoint vel;
    ofMesh velMesh;
    float angle;
    
    velMesh.setMode(OF_PRIMITIVE_LINES);
    
    for(int x=0; x<kinect.width; x+=3) {
        for(int y=0; y<kinect.height; y+=3) {
            vel = flowSolver.getVelAtPixel(x, y);
            ofVec3f p(x,y);
            ofVec3f vc = vel;
            vc.normalize();
            angle = atan2(vc.y, vc.x)+3.14159265;
            if(vel.length() < velocityMin || vel.length() >  velocityMax
               || angle < ofDegToRad(angleMin)
               || angle > ofDegToRad(angleMax)) {  // outside limits, no point drawing.
                continue;
            }
            float hue = angle/(3.14159265*2.0);
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

void ofApp::drawPointCloud() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                mesh.addColor(kinect.getColorAt(x,y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    glEnable(GL_DEPTH_TEST);
    mesh.drawVertices();
    glDisable(GL_DEPTH_TEST);
    ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::exit() {
    if (USE_KINECT2)
    {
        kinect2.close();
    }
    else
    {
        kinect.setCameraTiltAngle(0); // zero the tilt on exit
        kinect.close();
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
            
        case '>':
        case '.':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
        case '=':
            nearThreshold ++;
            if (nearThreshold > 255) nearThreshold = 255;
            break;
            
        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;
            
        case 'w':
            kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            break;
            
        case 'o':
            kinect.setCameraTiltAngle(angle); // go back to prev tilt
            kinect.open();
            break;
            
        case 'c':
            kinect.setCameraTiltAngle(0); // zero the tilt
            kinect.close();
            break;
            
        case '1':
            kinect.setLed(ofxKinect::LED_GREEN);
            break;
            
        case '2':
            kinect.setLed(ofxKinect::LED_YELLOW);
            break;
            
        case '3':
            kinect.setLed(ofxKinect::LED_RED);
            break;
            
        case '4':
            kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            break;
            
        case '5':
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;
            
        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
            break;
            
        case OF_KEY_UP:
            angle++;
            if(angle>30) angle=30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case OF_KEY_DOWN:
            angle--;
            if(angle<-30) angle=-30;
            kinect.setCameraTiltAngle(angle);
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