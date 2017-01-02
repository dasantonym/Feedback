#ifndef FEEDBACK_OSCRECEIVER_H
#define FEEDBACK_OSCRECEIVER_H

#include "ofMain.h"
#include "ofxOsc.h"

class OSCReceiver {
public:
    void setup();
    void update();

    float attack;
    float amp;
    float pitch;

private:
    ofxOscReceiver receiver;
};


#endif
