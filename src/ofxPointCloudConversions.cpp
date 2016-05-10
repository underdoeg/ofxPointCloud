#include "ofxPointCloudConversions.h"

void ofxPointCloud::toOf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ofMesh &mesh, ofVec3f scale){
    mesh.setMode(ofPrimitiveMode::OF_PRIMITIVE_POINTS);

    std::vector<ofVec3f> verts;
    verts.reserve(cloud->size());

    if(cloud->is_dense){
        for(auto p: *cloud){
            verts.push_back({p.x, p.y, p.z});
        }
    }else{
        for(auto p: *cloud){
            if(!isnan(p.x) && !isnan(p.y) && !isnan(p.z)){
                verts.push_back({p.x, p.y, p.z});
            }
        }
    }

    for(auto& v: verts){
        v *= scale;
    }

    if(mesh.getNumVertices() != verts.size()){
        mesh.clear();
        mesh.addVertices(verts);
    }else{
        for(size_t i=0; i<verts.size(); i++){
            mesh.setVertex(i, verts[i]);
        }
    }
}

void ofxPointCloud::toOf(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, ofMesh& mesh, ofVec3f scale){
    mesh.setMode(ofPrimitiveMode::OF_PRIMITIVE_POINTS);

    std::vector<ofVec3f> verts;
    verts.reserve(cloud->size());

    std::vector<ofFloatColor> colors;
    colors.reserve(cloud->size());

    if(cloud->is_dense){
        for(auto p: *cloud){
            verts.push_back({p.x, p.y, p.z});
            colors.push_back(ofColor(p.r, p.g, p.b, p.a));
        }
    }else{
        for(auto p: *cloud){
            if(!isnan(p.x) && !isnan(p.y) && !isnan(p.z)){
                verts.push_back({p.x, p.y, p.z});
                colors.push_back(ofColor(p.r, p.g, p.b, p.a));
                //colors.push_back(ofColor(p.r/255.f, p.g/255.f, p.b/255.f, p.a/255.f));
            }
        }
    }

    for(auto& v: verts){
        v *= scale;
    }

    if(mesh.getNumVertices() != verts.size()){
        mesh.clear();
        mesh.addVertices(verts);
        mesh.addColors(colors);
    }else{
        for(size_t i=0; i<verts.size(); i++){
            mesh.setVertex(i, verts[i]);
            mesh.setColor(i, colors[i]);
        }
    }
}
