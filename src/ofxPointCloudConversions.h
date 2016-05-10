#ifndef OFXPOINTCLOUDCONVERSIONS_H
#define OFXPOINTCLOUDCONVERSIONS_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ofMain.h"

namespace ofxPointCloud{

    void toOf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ofMesh& mesh, ofVec3f scale=ofVec3f(100, -100, 100));
    void toOf(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, ofMesh& mesh, ofVec3f scale=ofVec3f(100, -100, 100));
}

#endif // OFXPOINTCLOUDCONVERSIONS_H
