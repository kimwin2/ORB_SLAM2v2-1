#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ServerMap.h"
#include "ServerViewer.h"
#include "ORBParams.h"
#include "KeyFrame.h"
#include "MapPoint.h"

#include <ORB_SLAM2v2/MP.h>
#include <ORB_SLAM2v2/KF.h>
#include "std_msgs/String.h"
#include <boost/bind.hpp>
#include <thread>
#include <string>
#include "BoostArchiver.h"

#define NUMBER_OF_CLIENTS 1
#define INSERT 0
#define ERASE 1
#define UPDATE 2

namespace ORB_SLAM2{

class Communicator{
public:
    Communicator(){
        strSettingsFile = "/home/ros-slam/orb_ws/src/orb_slam/Examples/ROS/ORB_SLAM2/Asus.yaml";
        sm = new ServerMap();
        mpSMapDrawer = new MapDrawer(sm, strSettingsFile);
        mServerViewer = new ServerViewer(mpSMapDrawer, strSettingsFile);
        new thread(&ServerViewer::Run,mServerViewer);
        cout << "Communicator Created" << endl;
    }

    Communicator(char* str, ORBParams params){
        sm = new ServerMap();
        mpSMapDrawer = new MapDrawer(sm, str);
        mServerViewer = new ServerViewer(sm, params, mpSMapDrawer, str);
        new thread(&ServerViewer::Run,mServerViewer);
        cout << "Communicator Created" << endl;

    }

    void KeyFrameCallback(const ORB_SLAM2v2::KF::ConstPtr& msg){
        float twc[16] = {msg->Twc[0],msg->Twc[1],msg->Twc[2],msg->Twc[3],
        msg->Twc[4],msg->Twc[5],msg->Twc[6],msg->Twc[7],
        msg->Twc[8],msg->Twc[9],msg->Twc[10],msg->Twc[11],
        msg->Twc[12],msg->Twc[13],msg->Twc[14],msg->Twc[15]};
        float ow[3] = {msg->Ow[0],msg->Ow[1],msg->Ow[2]};
        cv::Mat Twc(4,4,CV_32F,twc);
        cv::Mat Ow(3,1,CV_32F,ow);
        vector<long unsigned int> cl(begin(msg->CovisibleList), end(msg->CovisibleList));
        vector<long unsigned int> lel(begin(msg->LoopEdgeList), end(msg->LoopEdgeList));
        if(msg->command == INSERT)
            sm->AddKeyFrame(new ServerKeyFrame(msg->mnId, Twc, Ow, cl, msg->Parent, lel));
        else if(msg->command == ERASE)
            sm->EraseKeyFrame(msg->mnId);
        else if(msg->command == UPDATE)
            sm->UpdateKeyFrame(new ServerKeyFrame(msg->mnId, Twc, Ow, cl, msg->Parent, lel));
    }

    void MapPointCallback(const ORB_SLAM2v2::MP::ConstPtr& msg){
        float ow[3] = {msg->mWorldPos[0],msg->mWorldPos[1],msg->mWorldPos[2]};
        cv::Mat Ow(3,1,CV_32F, ow);
        mnId = msg->mnId;
        if(msg->command == INSERT)
            sm->AddMapPoint(new ServerMapPoint(msg->UID, msg->mnId, Ow));
        else if(msg->command == ERASE)
            sm->EraseMapPoint(msg->UID);
        else if(msg->command == UPDATE)
            sm->UpdateMapPoint(new ServerMapPoint(msg->UID, msg->mnId, Ow));
    }

    void KeyFrameData(const std_msgs::String::ConstPtr& msg){
        stringstream sarray(msg->data);
        KeyFrame *kf = new KeyFrame();
        {
            boost::archive::binary_iarchive ia(sarray, boost::archive::no_header);
            ia >> kf;
        }
        //cout << "mnid : " << kf->mnId << endl;
    }

    void MapPointData(const std_msgs::String::ConstPtr& msg){

    }
    ServerMap *sm;
    MapDrawer *mpSMapDrawer;
    string strSettingsFile;
    ServerViewer *mServerViewer;
    ros::Subscriber kf_sub;
    ros::Subscriber mp_sub;

    int mnId = 0;

};

}

using namespace ORB_SLAM2;
using namespace std;

int main(int argc, char *argv[]){
    ros::init(argc, argv, "SERVER");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    ORBParams params;
    int ClientId=2;
    string mapBinaryPath;
    nh.param("ClientId", ClientId, ClientId);
    nh.param("mapBinaryPath", mapBinaryPath, mapBinaryPath);

    params.setMapBinaryPath(mapBinaryPath.c_str());

    Communicator ccom(argv[2], params);

    string kfName = "KEYFRAME" + to_string(ClientId);
    string mpName = "MAPPOINT" + to_string(ClientId);
    string kfDataName = "KEYFRAME_" + to_string(ClientId);
    string mpDataName = "MAPPOINT_" + to_string(ClientId);

    ros::Subscriber kf_sub = n.subscribe(kfName, 1000, &Communicator::KeyFrameCallback, &ccom);
    ros::Subscriber mp_sub = n.subscribe(mpName, 1000, &Communicator::MapPointCallback, &ccom);
    ros::Subscriber kf_data_sub = n.subscribe(kfDataName, 1000, &Communicator::KeyFrameData, &ccom);
    ros::Subscriber mp_data_sub = n.subscribe(mpDataName, 1000, &Communicator::MapPointData, &ccom);

    ros::spin();

    return 0;
}