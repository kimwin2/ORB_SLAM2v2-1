#include "ServerMap.h"

namespace ORB_SLAM2{

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

ServerKeyFrame::ServerKeyFrame(unsigned int mnid, cv::Mat twc, cv::Mat ow, vector<long unsigned int>  clist, int parentid, vector<long unsigned int>  llist){
    mnId = mnid;
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
}

void ServerMap::UpdateKeyFrame(ServerKeyFrame *skf){
    unique_lock<mutex> lock(mMutexMap);
    mspServerKeyFrames.erase(skf->mnId);
    mspServerKeyFrames.insert({skf->GetKeyFrameMnId(), skf});
}

map<unsigned int, ServerMapPoint*> ServerMap::GetAllMapPoints(){
    unique_lock<mutex> lock(mMutexMap);
    return mspServerMapPoints;
}

map<unsigned int, ServerKeyFrame*> ServerMap::GetAllKeyFrames(){
    unique_lock<mutex> lock(mMutexMap);
    return mspServerKeyFrames;
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