#ifndef SERVERMAP_H
#define SERVERMAP_H

#include <vector>
#include <set>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <ORB_SLAM2v2/MP.h>
#include <ORB_SLAM2v2/KF.h>
#include <ORB_SLAM2v2/Observation.h>

#include "BoostArchiver.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

using namespace std;

namespace ORB_SLAM2
{    

class ServerMapPoint
{
public:
    ServerMapPoint(){};
    ServerMapPoint(const ORB_SLAM2v2::MP::ConstPtr& msg);
    ServerMapPoint(unsigned int uid, unsigned int mnid, cv::Mat pos);

    cv::Mat GetWorldPos();
    unsigned int GetUID();

private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

public:
    unsigned int UID;
    unsigned int mnId;
    cv::Mat mWorldPos;

    unsigned int nNextId;
    int mnFirstKFid;
    int mnFirstFrame;
    int nObs;
    map<unsigned int, unsigned int> mObservations;
    cv::Mat mDescriptor;
    
    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    int mnVisible;
    int mnFound;
    float mfMinDistance;
    float mfMaxDistance;
};

class ServerKeyFrame
{
public:
    ServerKeyFrame(){};
    
    ServerKeyFrame(const ORB_SLAM2v2::KF::ConstPtr& msg);
    ServerKeyFrame(unsigned int mnid, cv::Mat tcw, cv::Mat twc, cv::Mat ow, vector<long unsigned int>  clist, int parentid, vector<long unsigned int>  llist);

    unsigned int GetKeyFrameMnId();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    vector<long unsigned int> GetCovisibleList();
    vector<long unsigned int> GetLoopEdgeList();
    vector<long unsigned int> GetMapPoints();
    int GetParent();
    void Swap(ServerKeyFrame *skf);

    unsigned int mnId;

private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

public:
    cv::Mat Twc;
    cv::Mat Tcw;
    cv::Mat Ow;
    vector<long unsigned int> CovisibleList;
    int parentId;
    vector<long unsigned int> LoopEdgeList;

    cv::Mat mDescriptors;
    DBoW2::FeatureVector mFeatVec;
    vector<cv::KeyPoint> mvKeysUn;
    vector<long unsigned int> mvpMapPoints;
    vector<float> mvuRight;
    vector<float> mvDepth;

    long unsigned int nNextId;
    long unsigned int mnFrameId;

    double mTimeStamp;

    // Grid (to speed up feature matching)
    int mnGridCols;
    int mnGridRows;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Scale
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;
};

class ServerMap
{
public:
    ServerMap():ConnectClient(true){};

    void AddMapPoint(ServerMapPoint *smp);
    void AddKeyFrame(ServerKeyFrame *skf);
    void EraseMapPoint(long unsigned int UID);
    void EraseKeyFrame(long unsigned int mnId);
    void UpdateMapPoint(ServerMapPoint *smp);
    void UpdateKeyFrame(ServerKeyFrame *skf);
    unsigned int GetKeyFrameOrigin();
    void Clear();
    void ConnectToClient();
    void DisconnectToClient();

    map<unsigned int, ServerMapPoint*> GetAllMapPoints();
    map<unsigned int, ServerKeyFrame*> GetAllKeyFrames();
    mutex mMutexMap;
    bool ConnectClient;

private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

private:
    map<unsigned int, ServerMapPoint*> mspServerMapPoints;
    map<unsigned int, ServerKeyFrame*> mspServerKeyFrames;
    unsigned int KeyFrameOrigin=0;
};

}

#endif