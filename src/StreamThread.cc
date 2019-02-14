#include "StreamThread.h"
#include<mutex>
#include "BoostArchiver.h"
#include<thread>

#include <ORB_SLAM2v2/KF.h>

namespace ORB_SLAM2{

SendClassToServer::SendClassToServer(ros::Publisher _data_pub, KeyFrame *_pKF, Map *pMap):data_pub(_data_pub), pKF(_pKF), mpMap(pMap){}

SendClassToServer::SendClassToServer(ros::NodeHandle _nh, int cid, Map *pMap):n(_nh), mpMap(pMap){
    string kfDataName = "KEYFRAME_" + to_string(cid);
    string mpDataName = "MAPPOINT_" + to_string(cid);
    kf_data_pub = n.advertise<ORB_SLAM2v2::KF>(kfDataName, 1000);
    mp_data_pub = n.advertise<std_msgs::String>(mpDataName, 1000);
    cout << kfDataName << endl;
    ClientId = cid;
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
    ORB_SLAM2v2::KF msg;

    vector<long unsigned int> cl;
    vector<long unsigned int> lel;
    cv::Mat cvt = pKF->GetPoseInverse();
    cv::Mat crt = pKF->GetCameraCenter();
    int pF = -1;
    if(pKF->GetParent())
        pF = pKF->GetParent()->mnId;

    vector<KeyFrame*> pK = pKF->GetCovisiblesByWeight(100);
    set<KeyFrame*> le = pKF->GetLoopEdges();
    if(!pK.empty()){
        for(vector<KeyFrame*>::iterator itx = pK.begin(); itx != pK.end(); itx++){
            unsigned int pkMnid = (*itx)->mnId;
            cl.push_back(pkMnid);
        }
    }

    if(!le.empty()){
        for(set<KeyFrame*>::iterator itx = le.begin(); itx != le.end(); itx++){
            lel.push_back((*itx)->mnId);
        }
    }

    msg.command = 0;
    msg.mClientId = GetClientId();
    msg.mnId = pKF->mnId;
    msg.Parent = pF;
    msg.CovisibleList.swap(cl);
    msg.LoopEdgeList.swap(lel);
    msg.Twc = {cvt.at<float>(0,0),cvt.at<float>(0,1),cvt.at<float>(0,2),cvt.at<float>(0,3),
    cvt.at<float>(1,0),cvt.at<float>(1,1),cvt.at<float>(1,2),cvt.at<float>(1,3),
    cvt.at<float>(2,0),cvt.at<float>(2,1),cvt.at<float>(2,2),cvt.at<float>(2,3),
    cvt.at<float>(3,0),cvt.at<float>(3,1),cvt.at<float>(3,2),cvt.at<float>(3,3)};

    msg.Ow = {crt.at<float>(0),crt.at<float>(1),crt.at<float>(2)};

    vector<MapPoint*> mvpMP = pKF->GetMapPointMatches();
    msg.mvpMapPoints.resize(mvpMP.size());
    for(int i = 0; i < mvpMP.size(); i++){
        if(mvpMP[i]==NULL){
            msg.mvpMapPoints[i] = 0;
        }else{
            msg.mvpMapPoints[i] = mvpMP[i]->UID;
        }        
    }

    ostringstream sarray;
    KeyFrame &kf = *pKF;
    cv::Mat desc = kf.mDescriptorsCopy;
    {
        boost::archive::binary_oarchive oa(sarray, boost::archive::no_header);
        oa << desc;
        oa << kf.mvKeysUn;
        oa << kf.mFeatVec;
    }
    msg.mDescriptors = sarray.str();
    kf_data_pub.publish(msg);
}

void SendClassToServer::Run(){
    mbFinished = false;
    mbFinishRequested = false;

    while(1){
        if(!mvpKF.empty()){
            pKF = mvpKF.front();
            RunKeyFrame();
            mvpKF.pop();
        }

        if(!mvpMP.empty()){
            
        }

        if(CheckFinish())
            break;

        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    SetFinish();
}


int SendClassToServer::GetClientId(){
    return ClientId;
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