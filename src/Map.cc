/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"
#include "StreamThread.h"
#include <thread>

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
    mpSendClassToServer->SetKeyFrame(pKF);

}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);

    ORB_SLAM2v2::MP msg;
    cv::Mat cvt = pMP->GetWorldPos();

    msg.command = 0;
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

    mp_pub.publish(msg);
}

void Map::AddKeyFrame(map<unsigned int, KeyFrame*> mspKFs){
    unique_lock<mutex> lock(mMutexMap);
    for(map<unsigned int,KeyFrame*>::iterator itx = mspKFs.begin(); itx != mspKFs.end(); itx++){
        mspKeyFrames.insert(itx->second);
    }
}

void Map::AddMapPoint(map<unsigned int, MapPoint*> mspMPs){
    unique_lock<mutex> lock(mMutexMap);
    for(map<unsigned int,MapPoint*>::iterator itx = mspMPs.begin(); itx != mspMPs.end(); itx++){
        mspMapPoints.insert(itx->second);
    }
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    ORB_SLAM2v2::MP msg;
    msg.command = 1;
    msg.UID = pMP->UID;
    mp_pub.publish(msg);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    ORB_SLAM2v2::KF msg;
    msg.command = 1;
    msg.mnId = pKF->mnId;
    kf_pub.publish(msg);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::UpdateKeyFrame(KeyFrame *pKF){
    unique_lock<mutex> lock(mMutexMap);

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

    msg.command = 2;
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
    kf_pub.publish(msg);
}

void Map::UpdateMapPoint(MapPoint *pMP){
    unique_lock<mutex> lock(mMutexMap);

    ORB_SLAM2v2::MP msg;
    cv::Mat cvt = pMP->GetWorldPos();

    msg.command = 2;
    msg.UID = pMP->UID;
    msg.mnId = pMP->mnId;
    msg.mWorldPos = {cvt.at<float>(0),cvt.at<float>(1),cvt.at<float>(2)};

    cv::Mat mDescriptor = pMP->GetDescriptor();
    for(int i=0; i<32; i++){
        msg.mDescriptor[i] = mDescriptor.at<unsigned char>(i);
    }
    mp_pub.publish(msg);
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::SetNodeHandle(ros::NodeHandle nh){
    n = nh;
}

void Map::SetClientId(int id){
    ClientId = id;
    string kfName = "KEYFRAME" + to_string(id);
    string mpName = "MAPPOINT" + to_string(id);

    kf_pub = n.advertise<ORB_SLAM2v2::KF>(kfName, 1000);
    mp_pub = n.advertise<ORB_SLAM2v2::MP>(mpName, 1000);
}

int Map::GetClientId(){
    return ClientId;
}

void Map::SetSendClassToServer(SendClassToServer* pSendClassToServer){
    mpSendClassToServer = pSendClassToServer;

}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

template<class Archive>
void Map::serialize(Archive &ar, const unsigned int version)
{
    // don't save mutex
    ar & mspMapPoints;
    ar & mvpKeyFrameOrigins;
    ar & mspKeyFrames;
    cout << "3" << endl;
    ar & mvpReferenceMapPoints;
    cout << "4" << endl;
    ar & mnMaxKFid & mnBigChangeIdx;
    cout << "5" << endl;
}
template void Map::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void Map::serialize(boost::archive::binary_oarchive&, const unsigned int);

} //namespace ORB_SLAM
