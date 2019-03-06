#ifndef SERVERVIEWER_H
#define SERVERVIEWER_H

#include "ServerMap.h"
#include "MapDrawer.h"
#include "ORBParams.h"
#include "ros/ros.h"
#include <pangolin/pangolin.h>

namespace ORB_SLAM2{

class ServerViewer
{
public:
    ServerViewer(MapDrawer *pSMapDrawer, const string &strSettingPath);
    ServerViewer(ServerMap *pSMap, ORBParams params, MapDrawer *pSMapDrawer, const string &strSettingPath);
    void Run();

private:
    MapDrawer* mpSMapDrawer;
    ServerMap* mpSMap;

    double mT;
    float mImageWidth, mImageHeight;
    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    int clientId;
    string mapBinaryPath;
    string mapOctomapPath;
    ros::Publisher map_pub;
    ros::Publisher octomap_pub;
    bool bConnect;
};

}

#endif