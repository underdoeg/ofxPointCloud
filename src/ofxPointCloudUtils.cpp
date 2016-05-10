#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_device_manager.h>
#include "ofxPointCloudUtils.h"


namespace ofxPointCloud{

//TODO: http://stackoverflow.com/questions/8988090/how-can-i-get-the-kinect-serial-number-with-openni

std::vector<GrabberDevice> getGrabberDevices(){

    auto deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance();

    auto infos = deviceManager->getConnectedDeviceInfos();
    for(size_t i=0; i<infos->size(); i++){
        auto info = infos->at(i);
        //ofLog() << info.name_;
        //ofLog() << info.product_id_;
        //ofLog() << info.uri_;
    }

    /*
    for(size_t i=0; i<deviceManager->getNumOfConnectedDevices(); i++){
        auto infos = deviceManager->getConnectedDeviceInfos();

        for(size_t j=0; j<infos->size(); j++){
            auto info = infos->at(j);

        }
    }
    */

    return {};
}

void printGrabberDevices(){

}

}
