#include "ServerMap.h"

namespace ORB_SLAM2{

ServerMapPoint::ServerMapPoint(const ORB_SLAM2v2::MP::ConstPtr& msg){
    float ow[3] = {msg->mWorldPos[0],msg->mWorldPos[1],msg->mWorldPos[2]};
    cv::Mat Ow(3,1,CV_32F, ow);
    UID = msg->UID;
    mnId = msg->mnId;
    mWorldPos = Ow.clone();

    nNextId = msg->nNextId;
    mnFirstKFid = msg->mnFirstKFid;
    mnFirstFrame = msg->mnFirstFrame;
    nObs = msg->nObs;
    vector<ORB_SLAM2v2::Observation> mObs = msg->mObservations;
    for(vector<ORB_SLAM2v2::Observation>::iterator itx = mObs.begin(); itx != mObs.end(); itx++){
        mObservations[itx->keyframe] = itx->idx;
    }

    mDescriptor.create(1,32,CV_8U);
    for(int i=0;i<32;i++){
        mDescriptor.at<unsigned char>(i) = msg->mDescriptor[i];
    }

    mTrackProjX = msg->mTrackProjX;
    mTrackProjY = msg->mTrackProjY;
    mTrackProjXR = msg->mTrackProjXR;
    mbTrackInView = msg->mbTrackInView;
    mnTrackScaleLevel = msg->mnTrackScaleLevel;
    mTrackViewCos = msg->mTrackViewCos;
    mnTrackReferenceForFrame = msg->mnTrackReferenceForFrame;
}

ServerMapPoint::ServerMapPoint(unsigned int uid, unsigned int mnid, cv::Mat pos){
    UID = uid;
    mnId = mnid;
    mWorldPos = pos.clone();
}

cv::Mat ServerMapPoint::GetWorldPos(){
    return mWorldPos.clone();
}

unsigned int ServerMapPoint::GetUID(){
    return UID;
}

ServerKeyFrame::ServerKeyFrame(const ORB_SLAM2v2::KF::ConstPtr& msg){
    stringstream sarray(msg->mDescriptors);
    {
        boost::archive::binary_iarchive ia(sarray, boost::archive::no_header);
        ia >> mDescriptors;
        ia >> mvKeysUn;
        ia >> mFeatVec;
    }

    float twc[16] = {msg->Twc[0],msg->Twc[1],msg->Twc[2],msg->Twc[3],
    msg->Twc[4],msg->Twc[5],msg->Twc[6],msg->Twc[7],
    msg->Twc[8],msg->Twc[9],msg->Twc[10],msg->Twc[11],
    msg->Twc[12],msg->Twc[13],msg->Twc[14],msg->Twc[15]};
    float tcw[16] = {msg->Tcw[0],msg->Tcw[1],msg->Tcw[2],msg->Tcw[3],
    msg->Tcw[4],msg->Tcw[5],msg->Tcw[6],msg->Tcw[7],
    msg->Tcw[8],msg->Tcw[9],msg->Tcw[10],msg->Tcw[11],
    msg->Tcw[12],msg->Tcw[13],msg->Tcw[14],msg->Tcw[15]};
    float ow[3] = {msg->Ow[0],msg->Ow[1],msg->Ow[2]};
    cv::Mat mTwc(4,4,CV_32F,twc);
    cv::Mat mTcw(4,4,CV_32F,tcw);
    cv::Mat mOw(3,1,CV_32F,ow);
    vector<long unsigned int> cl(begin(msg->CovisibleList), end(msg->CovisibleList));
    vector<long unsigned int> lel(begin(msg->LoopEdgeList), end(msg->LoopEdgeList));
    //vector<long unsigned int> mvpMP(begin(msg->mvpMapPoints), end(msg->mvpMapPoints));

    mnId = msg->mnId;
    Twc = mTwc.clone();
    Tcw = mTcw.clone();
    Ow = mOw.clone();
    parentId = msg->Parent;
    CovisibleList.swap(cl);
    LoopEdgeList.swap(lel);
    //mvpMapPoints.swap(mvpMP);

    nNextId = msg->nNextId;
    mnFrameId = msg->mnFrameId;
    mnGridCols = msg->mnGridCols;
    mnGridRows = msg->mnGridRows;
    mfGridElementWidthInv = msg->mfGridElementWidthInv;
    mfGridElementHeightInv = msg->mfGridElementHeightInv;
    mnTrackReferenceForFrame = msg->mnTrackReferenceForFrame;
    mnFuseTargetForKF = msg->mnFuseTargetForKF;
    mnBALocalForKF = msg->mnBALocalForKF;
    mnBAFixedForKF = msg->mnBAFixedForKF;
    mnLoopQuery = msg->mnLoopQuery;
    mnLoopWords = msg->mnLoopWords;
    mLoopScore = msg->mLoopScore;
    mnRelocQuery = msg->mnRelocQuery;
    mRelocScore = msg->mRelocScore;

    vector<float> mRight(begin(msg->mvuRight), end(msg->mvuRight));
    vector<float> mDepth(begin(msg->mvDepth), end(msg->mvDepth));
    mvuRight.swap(mRight);
    mvDepth.swap(mDepth);
}

ServerKeyFrame::ServerKeyFrame(unsigned int mnid, cv::Mat tcw, cv::Mat twc, cv::Mat ow, vector<long unsigned int>  clist, int parentid, vector<long unsigned int>  llist){
    mnId = mnid;
    Tcw = tcw.clone();
    Twc = twc.clone();
    Ow = ow.clone();
    CovisibleList.swap(clist);
    parentId = parentid;
    LoopEdgeList.swap(llist);
}

unsigned int ServerKeyFrame::GetKeyFrameMnId(){
    return mnId;
}

cv::Mat ServerKeyFrame::GetPoseInverse(){
    return Twc.clone();
}

cv::Mat ServerKeyFrame::GetCameraCenter(){
    return Ow.clone();
}

int ServerKeyFrame::GetParent(){
    return parentId;
}

vector<long unsigned int> ServerKeyFrame::GetCovisibleList(){
    return CovisibleList;
}

vector<long unsigned int> ServerKeyFrame::GetLoopEdgeList(){
    return LoopEdgeList;
}
/*
vector<long unsigned int> ServerKeyFrame::GetMapPoints(){
    return mvpMapPoints;
}
*/
void ServerKeyFrame::Swap(ServerKeyFrame *skf){
    Twc = skf->Twc.clone();
    Ow = skf->Ow.clone();
    CovisibleList = skf->GetCovisibleList();
    LoopEdgeList = skf->GetLoopEdgeList();
    //mvpMapPoints = skf->GetMapPoints();
}

void ServerMap::AddMapPoint(ServerMapPoint *smp){
    unique_lock<mutex> lock(mMutexMap);
    mspServerMapPoints.insert({smp->GetUID(), smp});
}

void ServerMap::EraseMapPoint(long unsigned int UID){
    unique_lock<mutex> lock(mMutexMap);
    mspServerMapPoints.erase(UID);
}

void ServerMap::UpdateMapPoint(ServerMapPoint *smp){
    unique_lock<mutex> lock(mMutexMap);
    mspServerMapPoints.erase(smp->GetUID());
    mspServerMapPoints.insert({smp->GetUID(), smp});
}

void ServerMap::AddKeyFrame(ServerKeyFrame *skf){
    unique_lock<mutex> lock(mMutexMap);
    mspServerKeyFrames.insert({skf->GetKeyFrameMnId(), skf});
    if(KeyFrameOrigin == 0)
        KeyFrameOrigin = skf->GetKeyFrameMnId();
}

void ServerMap::EraseKeyFrame(long unsigned int mnId){
    unique_lock<mutex> lock(mMutexMap);
    mspServerKeyFrames.erase(mnId);
    cout << "Erase KeyFrame ID : " << mnId << endl;
}

void ServerMap::UpdateKeyFrame(ServerKeyFrame *skf){
    unique_lock<mutex> lock(mMutexMap);
    if(mspServerKeyFrames.find(skf->mnId) != mspServerKeyFrames.end())
        mspServerKeyFrames.at(skf->mnId)->Swap(skf);
}

map<unsigned int, ServerMapPoint*> ServerMap::GetAllMapPoints(){
    unique_lock<mutex> lock(mMutexMap);
    return mspServerMapPoints;
}

map<unsigned int, ServerKeyFrame*> ServerMap::GetAllKeyFrames(){
    unique_lock<mutex> lock(mMutexMap);
    return mspServerKeyFrames;
}

unsigned int ServerMap::GetKeyFrameOrigin(){
    return KeyFrameOrigin;
}

void ServerMap::Clear(){
    unique_lock<mutex> lock(mMutexMap);
    mspServerKeyFrames.clear();
    mspServerMapPoints.clear();
}


template<class Archive>
void ServerKeyFrame::serialize(Archive &ar, const unsigned int version)
{
    // don't save mutex
    ar & Twc;
    ar & Ow;
    ar & CovisibleList;
    ar & parentId;
    ar & LoopEdgeList;
}
template void ServerKeyFrame::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void ServerKeyFrame::serialize(boost::archive::binary_oarchive&, const unsigned int);


template<class Archive>
void ServerMapPoint::serialize(Archive &ar, const unsigned int version)
{
    // don't save mutex
    ar & mnId;
    ar & mWorldPos;
}
template void ServerMapPoint::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void ServerMapPoint::serialize(boost::archive::binary_oarchive&, const unsigned int);

template<class Archive>
void ServerMap::serialize(Archive &ar, const unsigned int version)
{
    // don't save mutex
    ar & mspServerKeyFrames;
    ar & mspServerMapPoints;
}
template void ServerMap::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void ServerMap::serialize(boost::archive::binary_oarchive&, const unsigned int);

}//namespace ORB_SLAM2