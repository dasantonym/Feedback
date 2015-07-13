#include "ofMain.h"
#include "ofApp.h"
#include "ofAppGLFWWindow.h"

//========================================================================
int main( ){
    ofAppGLFWWindow window;
#ifdef linux
    ofSetupOpenGL(&window, 1280,800, OF_WINDOW);
#else
    ofSetupOpenGL(&window, 1280,800, OF_WINDOW);			// <-------- setup the GL context
#endif

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());

}
