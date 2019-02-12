#include "StreamThread.h"
#include<mutex>
#include "BoostArchiver.h"
#include<thread>

namespace ORB_SLAM2{

SendClassToServer::SendClassToServer(ros::Publisher _data_pub, KeyFrame *_pKF, Map *pMap):data_pub(_data_pub), pKF(_pKF), mpMap(pMap){}

SendClassToServer::SendClassToServer(ros::NodeHandle _nh, int cid, Map *pMap):n(_nh), mpMap(pMap){
    string kfDataName = "KEYFRAME_" + to_string(cid);
    string mpDataName = "MAPPOINT_" + to_string(cid);
    kf_data_pub = n.advertise<std_msgs::String>(kfDataName, 1000);
    mp_data_pub = n.advertise<std_msgs::String>(mpDataName, 1000);
}

void SendClassToServer::SetPublisher(ros::Publisher _data_pub){
    data_pub = _data_pub;
}

void SendClassToServer::SetKeyFrame(KeyFrame *_pKF){
    mvpKF.push(_pKF);
}

void SendClassToServer::SetMapPoint(MapPoint *_pMP){
    pMP = _pMP;
}


void SendClassToServer::RunKeyFrame(){
    std_msgs::String barray;
    ostringstream sarray;
    KeyFrame &kf = *pKF;
    cv::Mat desc = kf.mDescriptors.clone();
    {
        boost::archive::binary_oarchive oa(sarray, boost::archive::no_header);
        oa << desc;
    }
    barray.data = sarray.str();
    data_pub.publish(barray);
}

void SendClassToServer::Run(){
    mbFinished = false;

    while(1){
        if(!mvpKF.empty()){
            pKF = mvpKF.front();
            RunKeyFrame();
            mvpKF.pop();
        }

        if(CheckFinish())
            break;

        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    SetFinish();
}


void SendClassToServer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool SendClassToServer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void SendClassToServer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool SendClassToServer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


}