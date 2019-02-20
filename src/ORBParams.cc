#include "ORBParams.h"

namespace ORB_SLAM2
{

ORBParams::ORBParams(){
    publish_tf = false;
    publish_odom = false;
    mapping = false;
    build_octomap = false;
}

ORBParams::ORBParams(bool p, bool o, bool m, bool b){
    publish_tf = p;
    publish_odom = o;
    mapping = m;
    build_octomap = b;
}

void ORBParams::setPublishTf(bool p){
    publish_tf = p;
}
void ORBParams::setPublishOdom(bool o){
    publish_odom = o;
}
void ORBParams::setMapping(bool m){
    mapping = m;
}
void ORBParams::setBuildOctomap(bool b){
    build_octomap = b;
}

void ORBParams::setClientId(int id){
    ClientId = id;
}

void ORBParams::setNodeHandle(ros::NodeHandle nh){
    n = nh;
}

void ORBParams::setMapBinaryPath(const char* m){
    mapBinaryPath = m;
}
void ORBParams::setMapOctomapPath(const char* m){
    mapOctomapPath = m;

}
void ORBParams::setMapWorkingPath(const char* m){
    mapWorkingPath = m;

}
bool ORBParams::getPublishTf(){
    return publish_tf;
}
bool ORBParams::getpublishOdom(){
    return publish_odom;
}

void ORBParams::setMapPCLPath(const char* m){
    mapPCLPath = m;
}


bool ORBParams::getMapping(){
    return mapping;
}
bool ORBParams::getBuildOctomap(){
    return build_octomap;
}

const char* ORBParams::getMapBinaryPath(){
    return mapBinaryPath;
}

const char* ORBParams::getMapOctomapPath(){
    return mapOctomapPath;
}

const char* ORBParams::getMapPCLPath(){
    return mapPCLPath;
}
const char* ORBParams::getMapWorkingPath(){
    return mapWorkingPath;
}

int ORBParams::getClientId(){
    return ClientId;
}

ros::NodeHandle ORBParams::getNodeHandle(){
    return n;
}

}
