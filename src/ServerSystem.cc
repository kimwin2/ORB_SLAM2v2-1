#include "System.h"
#include "Converter.h"
#include "ServerMap.h"
#include "ServerViewer.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <cstdlib>
#include <ctime>
#include <thread>

#include <mutex>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <ORB_SLAM2v2/MP.h>
#include <ORB_SLAM2v2/KF.h>

using namespace std;
using namespace ORB_SLAM2;

int main(int argc, char *argv[]){

    ros::init(argc, argv, "TALKER");
    ros::NodeHandle n;
    ros::Publisher kf_pub = n.advertise<ORB_SLAM2v2::KF>("KEYFRAME", 1000);
    ros::Publisher mp_pub = n.advertise<ORB_SLAM2v2::MP>("MAPPOINT", 1000);

    Map *mpMap;
    KeyFrameDatabase *kfd;
    
    string strSettingsFile = "/home/ros-slam/orb_ws/src/orb_slam/Examples/ROS/ORB_SLAM2/Asus.yaml";

    ORB_SLAM2v2::MP msg;
    while(1){
        mp_pub.publish(msg);
        sleep(1);
        ros::spinOnce();
    }


    ifstream in("/home/ros-slam/map.bin",std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapfile, You need create it first!" << std::endl;
        return false;
    }

    {
        boost::archive::binary_iarchive ia(in, boost::archive::no_header);
        ia >> mpMap;
        ia >> kfd;
    }

    in.close();

    //MapDrawer *mpMapDrawer = new MapDrawer(new Map(), strSettingsFile);
    ServerMap *sm = new ServerMap();
    MapDrawer *mpSMapDrawer = new MapDrawer(sm, strSettingsFile);

    vector<KeyFrame*> pKF = mpMap->GetAllKeyFrames();
    vector<MapPoint*> pMP = mpMap->GetAllMapPoints();

    ServerViewer *mServerViewer = new ServerViewer(mpSMapDrawer, strSettingsFile);
    new thread(&ServerViewer::Run,mServerViewer);

    for(vector<KeyFrame*>::iterator iter = pKF.begin(); iter != pKF.end(); iter++){
        ORB_SLAM2v2::KF msg;
        unsigned int mnid = (*iter)->mnId;
        vector<KeyFrame*> pK = (*iter)->GetCovisiblesByWeight(100);
        set<KeyFrame*> le = (*iter)->GetLoopEdges();
        cv::Mat cvt = (*iter)->GetPoseInverse();
        
        float ttmp[16] = {cvt.at<float>(0,0),cvt.at<float>(0,1),cvt.at<float>(0,2),cvt.at<float>(0,3),
        cvt.at<float>(1,0),cvt.at<float>(1,1),cvt.at<float>(1,2),cvt.at<float>(1,3),
        cvt.at<float>(2,0),cvt.at<float>(2,1),cvt.at<float>(2,2),cvt.at<float>(2,3),
        cvt.at<float>(3,0),cvt.at<float>(3,1),cvt.at<float>(3,2),cvt.at<float>(3,3)};

        cv::Mat tmp(4,4,CV_32F, ttmp);

        vector<float> ftmp(begin(ttmp), end(ttmp));

        int pF = -1;
        if((*iter)->GetParent())
            pF = (*iter)->GetParent()->mnId;

        cv::Mat crt = (*iter)->GetCameraCenter();
        float ttat[3] = {crt.at<float>(0),crt.at<float>(1),crt.at<float>(2)};
        cv::Mat tmt(3,1,CV_32F, ttat);

        vector<long unsigned int> cl;
        vector<long unsigned int> lel;

        if(!pK.empty()){
            for(vector<KeyFrame*>::iterator itx = pK.begin(); itx != pK.end(); itx++){
                unsigned int pkMnid = (*itx)->mnId;
                cl.push_back(pkMnid);
            }
        }

        if(!le.empty()){
            for(set<KeyFrame*>::iterator itx = le.begin(); itx != le.end(); itx++){
                lel.push_back((*iter)->mnId);
            }
        }
        sm->AddKeyFrame(new ServerKeyFrame(mnid, tmp, tmt, cl, pF, lel));
        usleep(10000);

        msg.command = 0;
        msg.mClientId = 0;
        msg.mnId = mnid;
        msg.Parent = pF;
        msg.CovisibleList.swap(cl);
        msg.LoopEdgeList.swap(lel);
        msg.Twc = {cvt.at<float>(0,0),cvt.at<float>(0,1),cvt.at<float>(0,2),cvt.at<float>(0,3),
        cvt.at<float>(1,0),cvt.at<float>(1,1),cvt.at<float>(1,2),cvt.at<float>(1,3),
        cvt.at<float>(2,0),cvt.at<float>(2,1),cvt.at<float>(2,2),cvt.at<float>(2,3),
        cvt.at<float>(3,0),cvt.at<float>(3,1),cvt.at<float>(3,2),cvt.at<float>(3,3)};

        msg.Ow = {crt.at<float>(0),crt.at<float>(1),crt.at<float>(2)};


        kf_pub.publish(msg);
        ros::spinOnce();
    }

    for(vector<MapPoint*>::iterator iter = pMP.begin(); iter != pMP.end(); iter++){
        ORB_SLAM2v2::MP msg;
        unsigned int UID = rand();
        unsigned int mnId = (*iter)->mnId;
        cv::Mat cvt = (*iter)->GetWorldPos();
        float ttmp[3] = {cvt.at<float>(0),cvt.at<float>(1),cvt.at<float>(2)};
        cv::Mat tmp(3,1,CV_32F, ttmp);

        sm->AddMapPoint(new ServerMapPoint(UID, mnId, tmp));
        usleep(1000);

        msg.command = 0;
        msg.mClientId = 0;
        msg.UID = UID;
        msg.mnId = mnId;
        msg.mWorldPos = {cvt.at<float>(0),cvt.at<float>(1),cvt.at<float>(2)};

        mp_pub.publish(msg);
        ros::spinOnce();

    }

    sleep(10);

}
