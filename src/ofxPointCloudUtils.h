#ifndef OFXPOINTCLOUDUTILS_H
#define OFXPOINTCLOUDUTILS_H

#include <OpenNI.h>

#include "ofMain.h"

namespace ofxPointCloud{


struct GrabberDevice{
    std::string name;
    //std::string address;
    //std::string serial;
};

std::vector<GrabberDevice> getGrabberDevices();
void printGrabberDevices();

}

#endif // OFXPOINTCLOUDUTILS_H
