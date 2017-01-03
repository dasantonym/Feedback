#include "OSCReceiver.h"

void OSCReceiver::setup() {
    amp = .0f;
    attack = .0f;
    pitch = .0f;

    receiver.setup(5556);
}

void OSCReceiver::update() {
    while (receiver.hasWaitingMessages())
    {
        ofxOscMessage m;
        receiver.getNextMessage(m);

        if (m.getAddress() == "/growth/total")
        {
            attack = max(min(m.getArgAsFloat(0) / 10.f, 1.f), .0f);
        }
    }
}