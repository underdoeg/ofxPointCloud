#ifndef OFXPOINTCLOUD_H
#define OFXPOINTCLOUD_H

#include <pcl/io/pcd_io.h>
#include "ofxPointCloudGrabber.h"
#include <pcl/visualization/cloud_viewer.h>
#include "ofxPointCloudUtils.h"
#include "ofxPointCloudConversions.h"
#include "ofMain.h"

//

#define OFX_POINTCLOUD_DEPTH_IMG 0x01
#define OFX_POINTCLOUD_COLOR_IMG 0x02
#define OFX_POINTCLOUD_MESH 0x03

namespace ofxPointCloud{




template<typename Type>
class PointCloudBase{
public:
    PointCloudBase():
        pclCloud(new pcl::PointCloud<Type>()),
        grabber(nullptr),
        bMeshDirty(false),
        bDepthPixelsDirty(false),
        bImgDirty(false),
        scale(10, -10, 10),
        bIsGrabbing(false),
        isColor(typeid(Type) == typeid(pcl::PointXYZRGBA)){
        //pcl::console::setVerbosityLevel(pcl::console::L_WARN);
    }

    ~PointCloudBase(){
        stopGrabbing();
    }

    void set(const typename pcl::PointCloud<Type>::ConstPtr& other){
        pcl::copyPointCloud(*other, *pclCloud);
        setDirty();
    }

    void startGrabbing(std::string deviceId=""){
        if(bIsGrabbing){
            ofLogWarning("ofxPointCloud") << "Grabber already started " << deviceId;
            return;
        }

        try {
            grabber = new GrabberPclOpenNI2<Type>(deviceId);
            grabber->start();
            bIsGrabbing = true;
        } catch (...) {
            ofLogError("ofxPointCloud") << "Error opening device";
        }
    }

    bool isGrabbing(){
        return bIsGrabbing;
    }

    void stopGrabbing(){
        if(!grabber)
            return;
        ofLogNotice("ofxPointCloud") << "Stopping grabber";
        grabber->stop();
        grabber->waitForThread();
        bIsGrabbing = false;
        delete grabber;
        grabber = nullptr;
    }

    bool loadPCD(std::string path){
        pcl::io::loadPCDFile(path, *pclCloud);
    }

    bool savePCD(std::string path){
        pcl::io::savePCDFile(path, *pclCloud);
    }

    const ofMesh& convertToMesh(){
        toOf(pclCloud, mesh, scale);
        bMeshDirty = false;
        //mesh.save("test.ply");
        return mesh;
    }

    const ofMesh& getMesh(){
        updateFromGrabber();
        if(bMeshDirty)
            return convertToMesh();
        return mesh;
    }

    const ofImage& getImage(){
        updateFromGrabber();
        return image;
    }

    void draw(){
        //if(getMesh().getNumVertices() == 0)
        //  return;
        //ofPushMatrix();
        //ofTranslate(-mesh.getCentroid());
        getMesh().drawVertices();
        //ofPopMatrix();
    }

    void drawImage(float x=0, float y=0){
        const ofImage& img = getImage();
        if(!img.isAllocated())
            return;
        img.draw(x, y);
    }

    typename pcl::PointCloud<Type>::Ptr pclCloud;

private:
    void setDirty(){
        bMeshDirty = bDepthPixelsDirty = bImgDirty = true;
    }

    void updateFromGrabber(){
        if(!isGrabbing())
            return;
        if(!grabber->isFrameNew())
            return;

        set(grabber->getCloud());

        if(isColor)
            image.setFromPixels(grabber->getPixels());

        grabber->frameHandled();
    }

    Grabber<Type>* grabber;

    bool bMeshDirty;
    ofMesh mesh;

    bool bDepthPixelsDirty;
    ofFloatPixels depthPixels;

    bool bImgDirty;
    ofImage image;

    ofVec3f scale;

    bool bIsGrabbing;

    bool isColor;
};


using PointCloud = PointCloudBase<pcl::PointXYZ>;
//using PointCloudRGB = PointCloudBase<pcl::PointXYZRGB>;
using PointCloudRGBA = PointCloudBase<pcl::PointXYZRGBA>;


}

#endif // OFXPOINTCLOUD_H
