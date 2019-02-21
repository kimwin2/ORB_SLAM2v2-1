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
    mp_data_pub = n.advertise<ORB_SLAM2v2::MP>(mpDataName, 1000);
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
    mvpMP.push(_pMP);
}

void SendClassToServer::EraseKeyFrame(KeyFrame *_pKF){
    mvpDKF.push(_pKF);
}

void SendClassToServer::EraseMapPoint(MapPoint *_pMP){
    mvpDMP.push(_pMP);
}

void SendClassToServer::UpdateKeyFrame(KeyFrame *_pKF){
    mvpUKF.push(_pKF);
}

void SendClassToServer::UpdateMapPoint(MapPoint *_pMP){
    mvpUMP.push(_pMP);
}


void SendClassToServer::RunKeyFrame(int command){
    ORB_SLAM2v2::KF msg;

    vector<long unsigned int> cl;
    vector<long unsigned int> lel;
    cv::Mat ctv = pKF->GetPose();
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

    msg.command = command;
    msg.mClientId = GetClientId();
    msg.mnId = pKF->mnId;
    msg.Parent = pF;
    msg.CovisibleList.swap(cl);
    msg.LoopEdgeList.swap(lel);
    msg.Tcw = {ctv.at<float>(0,0),ctv.at<float>(0,1),ctv.at<float>(0,2),ctv.at<float>(0,3),
    ctv.at<float>(1,0),ctv.at<float>(1,1),ctv.at<float>(1,2),ctv.at<float>(1,3),
    ctv.at<float>(2,0),ctv.at<float>(2,1),ctv.at<float>(2,2),ctv.at<float>(2,3),
    ctv.at<float>(3,0),ctv.at<float>(3,1),ctv.at<float>(3,2),ctv.at<float>(3,3)};
    msg.Twc = {cvt.at<float>(0,0),cvt.at<float>(0,1),cvt.at<float>(0,2),cvt.at<float>(0,3),
    cvt.at<float>(1,0),cvt.at<float>(1,1),cvt.at<float>(1,2),cvt.at<float>(1,3),
    cvt.at<float>(2,0),cvt.at<float>(2,1),cvt.at<float>(2,2),cvt.at<float>(2,3),
    cvt.at<float>(3,0),cvt.at<float>(3,1),cvt.at<float>(3,2),cvt.at<float>(3,3)};

    msg.Ow = {crt.at<float>(0),crt.at<float>(1),crt.at<float>(2)};

    msg.nNextId = pKF->nNextId;
    msg.mnFrameId = pKF->mnFrameId;
    msg.mTimeStamp = pKF->mTimeStamp;
    msg.mnGridCols = pKF->mnGridCols;
    msg.mnGridRows = pKF->mnGridRows;
    msg.mfGridElementWidthInv = pKF->mfGridElementWidthInv;
    msg.mfGridElementHeightInv = pKF->mfGridElementHeightInv;
    msg.mnTrackReferenceForFrame = pKF->mnTrackReferenceForFrame;
    msg.mnFuseTargetForKF = pKF->mnFuseTargetForKF;
    msg.mnBALocalForKF = pKF->mnBALocalForKF;
    msg.mnBAFixedForKF = pKF->mnBAFixedForKF;
    msg.mnLoopQuery = pKF->mnLoopQuery;
    msg.mnLoopWords = pKF->mnLoopWords;
    msg.mLoopScore = pKF->mLoopScore;
    msg.mnRelocQuery = pKF->mnRelocQuery;
    msg.mnRelocWords = pKF->mnRelocWords;
    msg.mRelocScore = pKF->mRelocScore;
/*
    vector<MapPoint*> mvpMP = pKF->GetMapPointMatches();
    msg.mvpMapPoints.resize(mvpMP.size());
    for(int i = 0; i < mvpMP.size(); i++){
        if(mvpMP[i]==NULL){
            msg.mvpMapPoints[i] = 0;
        }else{
            msg.mvpMapPoints[i] = mvpMP[i]->UID;
        }        
    }
*/
    if(command == 0){
        for(int i = 0 ; i < pKF->mvuRight.size(); i++){
            msg.mvuRight.push_back(pKF->mvuRight[i]);
        }
        for(int i = 0; i < pKF->mvDepth.size(); i++){
            msg.mvDepth.push_back(pKF->mvDepth[i]);
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
    }
    kf_data_pub.publish(msg);
}

void SendClassToServer::RunMapPoint(int command){
    ORB_SLAM2v2::MP msg;
    cv::Mat cvt = pMP->GetWorldPos();

    msg.command = command;
    msg.UID = pMP->UID;
    msg.mnId = pMP->mnId;
    msg.mWorldPos = {cvt.at<float>(0),cvt.at<float>(1),cvt.at<float>(2)};

    map<KeyFrame*,size_t> mOb = pMP->GetObservations();
    for(map<KeyFrame*,size_t>::iterator itx = mOb.begin(); itx != mOb.end(); itx++){
        ORB_SLAM2v2::Observation ob;
        ob.keyframe = (*itx).first->mnId;
        ob.idx = (*itx).second;
        msg.mObservations.push_back(ob);
    }
    cv::Mat mDescriptor = pMP->GetDescriptor();
    for(int i=0; i<32; i++){
        msg.mDescriptor[i] = mDescriptor.at<unsigned char>(i);
    }

    msg.nNextId = pMP->nNextId;
    msg.mnFirstKFid = pMP->mnFirstKFid;
    msg.mnFirstFrame = pMP->mnFirstFrame;
    msg.nObs = pMP->nObs;

    mp_data_pub.publish(msg);
}

void SendClassToServer::RunMapPoint(MapPoint* _pMP, int command){
    ORB_SLAM2v2::MP msg;
    cv::Mat cvt = _pMP->GetWorldPos();

    msg.command = command;
    msg.UID = _pMP->UID;
    msg.mnId = _pMP->mnId;
    msg.mWorldPos = {cvt.at<float>(0),cvt.at<float>(1),cvt.at<float>(2)};

    map<KeyFrame*,size_t> mOb = _pMP->GetObservations();
    for(map<KeyFrame*,size_t>::iterator itx = mOb.begin(); itx != mOb.end(); itx++){
        ORB_SLAM2v2::Observation ob;
        ob.keyframe = (*itx).first->mnId;
        ob.idx = (*itx).second;
        msg.mObservations.push_back(ob);
    }
    cv::Mat mDescriptor = _pMP->GetDescriptor();
    for(int i=0; i<32; i++){
        msg.mDescriptor[i] = mDescriptor.at<unsigned char>(i);
    }

    msg.nNextId = _pMP->nNextId;
    msg.mnFirstKFid = _pMP->mnFirstKFid;
    msg.mnFirstFrame = _pMP->mnFirstFrame;
    msg.nObs = _pMP->nObs;

    msg.mTrackProjX = _pMP->mTrackProjX;
    msg.mTrackProjY = _pMP->mTrackProjY;
    msg.mTrackProjXR = _pMP->mTrackProjXR;
    msg.mbTrackInView = _pMP->mbTrackInView;
    msg.mnTrackScaleLevel = _pMP->mnTrackScaleLevel;
    msg.mTrackViewCos = _pMP->mTrackViewCos;
    msg.mnTrackReferenceForFrame = _pMP->mnTrackReferenceForFrame;

    mp_data_pub.publish(msg);
}

void SendClassToServer::Run(){
    mbFinished = false;
    mbFinishRequested = false;

    while(1){
        if(!mvpKF.empty()){
            pKF = mvpKF.front();
            RunKeyFrame(0);
            mvpKF.pop();
        }

        if(!mvpMP.empty()){
            pMP = mvpMP.front();
            RunMapPoint(0);
            mvpMP.pop();
        }

        if(!mvpUKF.empty()){
            pKF = mvpUKF.front();
            RunKeyFrame(2);
            mvpUKF.pop();
        }

        if(!mvpUMP.empty()){
            pMP = mvpUMP.front();
            RunMapPoint(2);
            mvpUMP.pop();
        }

        if(!mvpDKF.empty()){
            pKF = mvpDKF.front();
            RunKeyFrame(1);
            mvpDKF.pop();
        }

        if(!mvpDMP.empty()){
            pMP = mvpDMP.front();
            RunMapPoint(1);
            mvpDMP.pop();
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