/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "Odometry/OdometryORBSLAM2.h"
#include "Odometry/OdometryInfo.h"
#include "Odometry/util2d.h"
#include "Odometry/util3d_transforms.h"
//#include "utilite/ULogger.h"
#include "utilite/UTimer.h"
#include "utilite/UStl.h"
#include "utilite/UDirectory.h"
#include <pcl/common/transforms.h>
#include <opencv2/imgproc/types_c.h>

#include <System.h>
#include <thread>

using namespace std;

namespace ORB_SLAM2 {
// Override original Tracking object to comment all rendering stuff
class Tracker: public Tracking
{
public:
	Tracker(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
	             KeyFrameDatabase* pKFDB, const std::string &strSettingPath, const int sensor, long unsigned int maxFeatureMapSize) :
	            	 Tracking(pSys, pVoc, pFrameDrawer, pMapDrawer, pMap, pKFDB, strSettingPath, sensor),
	            	 maxFeatureMapSize_(maxFeatureMapSize)
	{

	}
private:
	long unsigned int maxFeatureMapSize_;

protected:
	void Track()
	{
	    if(mState==NO_IMAGES_YET)
	    {
	        mState = NOT_INITIALIZED;
	    }

	    mLastProcessedState=mState;

	    // Get Map Mutex -> Map cannot be changed
	    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

	    if(mState==NOT_INITIALIZED)
	    {
	       // if(mSensor==System::STEREO || mSensor==System::RGBD)
	            StereoInitialization();
	        //else
	        //    MonocularInitialization();

	        //mpFrameDrawer->Update(this);

	        if(mState!=OK)
	            return;
	    }
	    else
	    {
	        // System is initialized. Track Frame.
	        bool bOK;

	        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
	        if(!mbOnlyTracking)
	        {
	            // Local Mapping is activated. This is the normal behaviour, unless
	            // you explicitly activate the "only tracking" mode.

	            if(mState==OK || mState==LOST)
	            {
	                // Local Mapping might have changed some MapPoints tracked in last frame
	                CheckReplacedInLastFrame();

	                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
	                {
	                    bOK = TrackReferenceKeyFrame();
	                }
	                else
	                {
	                    bOK = TrackWithMotionModel();
	                    if(!bOK)
	                        bOK = TrackReferenceKeyFrame();
	                }
	                if(bOK)
	                {
	                	mState=OK;
	                }
	            }
	            else
	            {
	                bOK = Relocalization();
	            }
	        }
	        else
	        {
	            // Localization Mode: Local Mapping is deactivated

	            if(mState==LOST)
	            {
	                bOK = Relocalization();
	            }
	            else
	            {
	                if(!mbVO)
	                {
	                    // In last frame we tracked enough MapPoints in the map

	                    if(!mVelocity.empty())
	                    {
	                        bOK = TrackWithMotionModel();
	                    }
	                    else
	                    {
	                        bOK = TrackReferenceKeyFrame();
	                    }
	                }
	                else
	                {
	                    // In last frame we tracked mainly "visual odometry" points.

	                    // We compute two camera poses, one from motion model and one doing relocalization.
	                    // If relocalization is sucessfull we choose that solution, otherwise we retain
	                    // the "visual odometry" solution.

	                    bool bOKMM = false;
	                    bool bOKReloc = false;
	                    std::vector<MapPoint*> vpMPsMM;
	                    std::vector<bool> vbOutMM;
	                    cv::Mat TcwMM;
	                    if(!mVelocity.empty())
	                    {
	                        bOKMM = TrackWithMotionModel();
	                        vpMPsMM = mCurrentFrame.mvpMapPoints;
	                        vbOutMM = mCurrentFrame.mvbOutlier;
	                        TcwMM = mCurrentFrame.mTcw.clone();
	                    }
	                    bOKReloc = Relocalization();

	                    if(bOKMM && !bOKReloc)
	                    {
	                        mCurrentFrame.SetPose(TcwMM);
	                        mCurrentFrame.mvpMapPoints = vpMPsMM;
	                        mCurrentFrame.mvbOutlier = vbOutMM;

	                        if(mbVO)
	                        {
	                            for(int i =0; i<mCurrentFrame.N; i++)
	                            {
	                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
	                                {
	                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
	                                }
	                            }
	                        }
	                    }
	                    else if(bOKReloc)
	                    {
	                        mbVO = false;
	                    }

	                    bOK = bOKReloc || bOKMM;
	                }
	            }
	        }

	        mCurrentFrame.mpReferenceKF = mpReferenceKF;

	        // If we have an initial estimation of the camera pose and matching. Track the local map.
	        if(!mbOnlyTracking)
	        {
	            if(bOK)
	                bOK = TrackLocalMap();
	        }
	        else
	        {
	            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
	            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
	            // the camera we will use the local map again.
	            if(bOK && !mbVO)
	                bOK = TrackLocalMap();
	        }

	        if(bOK)
	            mState = OK;
	        else
	            mState=LOST;

	        // Update drawer
	        //mpFrameDrawer->Update(this);

	        // If tracking were good, check if we insert a keyframe
	        if(bOK)
	        {
	            // Update motion model
	            if(!mLastFrame.mTcw.empty())
	            {
	                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
	                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
	                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
	                mVelocity = mCurrentFrame.mTcw*LastTwc;
	            }
	            else
	                mVelocity = cv::Mat();

	            //mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

	            // Clean VO matches
	            for(int i=0; i<mCurrentFrame.N; i++)
	            {
	                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
	                if(pMP)
	                    if(pMP->Observations()<1)
	                    {
	                        mCurrentFrame.mvbOutlier[i] = false;
	                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
	                    }
	            }

	            // Delete temporal MapPoints
	            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
	            {
	                MapPoint* pMP = *lit;
	                delete pMP;
	            }
	            mlpTemporalPoints.clear();

	            // Check if we need to insert a new keyframe
	            if(NeedNewKeyFrame())
	            {
	                CreateNewKeyFrame();
	            }

	            if(maxFeatureMapSize_ > 0)
				{
					//limit size of the feature map, keep last X recent ones
					if(mpMap->KeyFramesInMap()>1 && mpMap->MapPointsInMap()>maxFeatureMapSize_)
					{
						std::vector<KeyFrame*> kfs = mpMap->GetAllKeyFrames();
						std::map<long unsigned int, KeyFrame*> kfsSorted;
						for(unsigned int i=1; i<kfs.size(); ++i)
						{
							kfsSorted.insert(std::make_pair(kfs[i]->mnId, kfs[i]));
						}
						KeyFrame * lastFrame = kfsSorted.rbegin()->second;
						std::vector<MapPoint*> mapPoints = mpMap->GetAllMapPoints();
						std::map<long unsigned int, MapPoint*> mapPointsSorted;
						for(unsigned int i=0; i<mapPoints.size(); ++i)
						{
							mapPointsSorted.insert(std::make_pair(mapPoints[i]->mnId, mapPoints[i]));
						}

						for(std::map<long unsigned int, MapPoint*>::iterator iter=mapPointsSorted.begin();
							iter != mapPointsSorted.end() && mpMap->MapPointsInMap()>maxFeatureMapSize_;
							++iter)
						{
							if(!iter->second->IsInKeyFrame(lastFrame))
							{
								// FIXME: Memory leak: ORB_SLAM2 doesn't delete after removing from the map...
								// Not sure when it is safe to delete it, as if I delete just
								// after setting the bad flag, the app crashes.
								iter->second->SetBadFlag();
							}
						}
						// remove kfs without observations
						for(std::map<long unsigned int, KeyFrame*>::iterator iter=kfsSorted.begin();
							iter != kfsSorted.end();
							++iter)
						{
							if(iter->second!=lastFrame && iter->second->GetMapPoints().size()==0)
							{
								// FIXME: Memory leak: ORB_SLAM2 doesn't delete after removing from the map...
								// Not sure when it is safe to delete it, as if I delete just
								// after setting the bad flag, the app crashes.
								iter->second->SetErase();
							}
							else
							{
								break;
							}
						}
					}
				}

	            // We allow points with high innovation (considererd outliers by the Huber Function)
	            // pass to the new keyframe, so that bundle adjustment will finally decide
	            // if they are outliers or not. We don't want next frame to estimate its position
	            // with those points so we discard them in the frame.
	            for(int i=0; i<mCurrentFrame.N;i++)
	            {
	                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
	                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
	            }
	        }

	        // Reset if the camera get lost soon after initialization
	        if(mState==LOST)
	        {
	            //if(mpMap->KeyFramesInMap()<=5)
	            {
	               	printf("Track lost...");
	                 return;

	            }
	        }

	        if(!mCurrentFrame.mpReferenceKF)
	            mCurrentFrame.mpReferenceKF = mpReferenceKF;

	        mLastFrame = Frame(mCurrentFrame);
	    }

	    // Store frame pose information to retrieve the complete camera trajectory afterwards.
	    if(!mCurrentFrame.mTcw.empty())
	    {
	        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
	        mlRelativeFramePoses.push_back(Tcr);
	        mlpReferences.push_back(mpReferenceKF);
	        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
	        mlbLost.push_back(mState==LOST);
	    }
	    else
	    {
	        // This can happen if tracking is lost
	        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
	        mlpReferences.push_back(mlpReferences.back());
	        mlFrameTimes.push_back(mlFrameTimes.back());
	        mlbLost.push_back(mState==LOST);
	    }
	}

	void StereoInitialization()
	{
	    if(mCurrentFrame.N>500)
	    {
	        // Set Frame pose to the origin
	        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

	        // Create KeyFrame
	        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

	        // Insert KeyFrame in the map
	        mpMap->AddKeyFrame(pKFini);

	        // Create MapPoints and asscoiate to KeyFrame
	        for(int i=0; i<mCurrentFrame.N;i++)
	        {
	            float z = mCurrentFrame.mvDepth[i];
	            if(z>0)
	            {
	                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
	                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
	                pNewMP->AddObservation(pKFini,i);
	                pKFini->AddMapPoint(pNewMP,i);
	                pNewMP->ComputeDistinctiveDescriptors();
	                pNewMP->UpdateNormalAndDepth();
	                mpMap->AddMapPoint(pNewMP);

	                mCurrentFrame.mvpMapPoints[i]=pNewMP;
	            }
	        }

	        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

	        mpLocalMapper->InsertKeyFrame(pKFini);

	        mLastFrame = Frame(mCurrentFrame);
	        mnLastKeyFrameId=mCurrentFrame.mnId;
	        mpLastKeyFrame = pKFini;

	        mvpLocalKeyFrames.push_back(pKFini);
	        mvpLocalMapPoints=mpMap->GetAllMapPoints();
	        mpReferenceKF = pKFini;
	        mCurrentFrame.mpReferenceKF = pKFini;

	        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

	        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

	        //mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

	        mState=OK;
	    }
	}

public:
	cv::Mat GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
	{
	    mImGray = imRectLeft;
	    cv::Mat imGrayRight = imRectRight;

	    if(mImGray.channels()==3)
	    {
	        if(mbRGB)
	        {
	            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
	        }
	        else
	        {
	            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
	        }
	    }
	    else if(mImGray.channels()==4)
	    {
	        if(mbRGB)
	        {
	            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
	        }
	        else
	        {
	            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
	        }
	    }
	    if(imGrayRight.channels()==3)
		{
			if(mbRGB)
			{
				cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
			}
			else
			{
				cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
			}
		}
		else if(imGrayRight.channels()==4)
		{
			if(mbRGB)
			{
				cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
			}
			else
			{
				cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
			}
		}

	    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

	    Track();

	    return mCurrentFrame.mTcw.clone();
	}

	cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
	{
	    mImGray = imRGB;
	    cv::Mat imDepth = imD;

	    if(mImGray.channels()==3)
	    {
	        if(mbRGB)
	            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
	        else
	            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
	    }
	    else if(mImGray.channels()==4)
	    {
	        if(mbRGB)
	            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
	        else
	            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
	    }

	    //UASSERT(imDepth.type()==CV_32F);

	    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

	    Track();

	    return mCurrentFrame.mTcw.clone();
	}
};

class LoopCloser: public LoopClosing
{
public:
	LoopCloser(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale) :
		LoopClosing(pMap, pDB, pVoc, bFixScale)
	{
	}

public:
	void RunNoLoop()
	{
		mbFinished =false;

		while(1)
		{
			// just clear the buffer
			{
				unique_lock<mutex> lock(mMutexLoopQueue);
				mlpLoopKeyFrameQueue.clear();
			}

			ResetIfRequested();

			if(CheckFinish())
				break;

			usleep(1000000); // 1 sec
		}

		SetFinish();
	}
};

} // namespace ORB_SLAM2
