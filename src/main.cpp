#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main( ){
    ofGLWindowSettings settings;
    //settings.setGLVersion(3,3);
    settings.setGLVersion(1,2);
    settings.width = 1920;
    settings.height = 1080;
    ofCreateWindow(settings);
    ofRunApp(new ofApp());
}