#ifndef SERVERMAP_H
#define SERVERMAP_H

#include <vector>
#include <set>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "BoostArchiver.h"

using namespace std;

namespace ORB_SLAM2
{    

class ServerMapPoint
{
public:
    ServerMapPoint(){};
    ServerMapPoint(unsigned int uid, unsigned int mnid, cv::Mat pos);

    cv::Mat GetWorldPos();
    unsigned int GetUID();

private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

private:
    unsigned int UID;
    unsigned int mnId;
    cv::Mat mWorldPos;

};

class ServerKeyFrame
{
public:
    ServerKeyFrame(){};
    ServerKeyFrame(unsigned int mnid, cv::Mat twc, cv::Mat ow, vector<long unsigned int>  clist, int parentid, vector<long unsigned int>  llist);

    unsigned int GetKeyFrameMnId();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    vector<long unsigned int> GetCovisibleList();
    vector<long unsigned int> GetLoopEdgeList();
    int GetParent();

    unsigned int mnId;

private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

private:
    cv::Mat Twc;
    cv::Mat Ow;
    vector<long unsigned int> CovisibleList;
    int parentId;
    vector<long unsigned int> LoopEdgeList;
};

class ServerMap
{
public:
    ServerMap(){};

    void AddMapPoint(ServerMapPoint *smp);
    void AddKeyFrame(ServerKeyFrame *skf);
    void EraseMapPoint(long unsigned int UID);
    void EraseKeyFrame(long unsigned int mnId);
    void UpdateMapPoint(ServerMapPoint *smp);
    void UpdateKeyFrame(ServerKeyFrame *skf);
    void Clear();

    map<unsigned int, ServerMapPoint*> GetAllMapPoints();
    map<unsigned int, ServerKeyFrame*> GetAllKeyFrames();
    mutex mMutexMap;

private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

private:
    map<unsigned int, ServerMapPoint*> mspServerMapPoints;
    map<unsigned int, ServerKeyFrame*> mspServerKeyFrames;
};

}

#endif