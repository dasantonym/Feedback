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
}