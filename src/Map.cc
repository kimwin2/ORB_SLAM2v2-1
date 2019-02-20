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
    mpSendClassToServer = static_cast<SendClassToServer*>(NULL);
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
    if(mpSendClassToServer != NULL)
        mpSendClassToServer->SetKeyFrame(pKF);

}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
    if(mpSendClassToServer != NULL)
        mpSendClassToServer->RunMapPoint(pMP, 0);
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
    if(mpSendClassToServer != NULL)
        mpSendClassToServer->RunMapPoint(pMP,1);
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if(mpSendClassToServer != NULL)
        mpSendClassToServer->EraseKeyFrame(pKF);
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::UpdateKeyFrame(KeyFrame *pKF){
    unique_lock<mutex> lock(mMutexMap);
    if(mpSendClassToServer != NULL)
        mpSendClassToServer->UpdateKeyFrame(pKF);
}

void Map::UpdateMapPoint(MapPoint *pMP){
    unique_lock<mutex> lock(mMutexMap);
    if(mpSendClassToServer != NULL)
        mpSendClassToServer->RunMapPoint(pMP,2);
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
    ar & mvpReferenceMapPoints;
    ar & mnMaxKFid & mnBigChangeIdx;
}
template void Map::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void Map::serialize(boost::archive::binary_oarchive&, const unsigned int);

} //namespace ORB_SLAM
