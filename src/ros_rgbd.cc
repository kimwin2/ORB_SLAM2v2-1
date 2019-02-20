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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include "include/System.h"
#include "ORBParams.h"

#include <std_srvs/Empty.h>
#include <Converter.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <ORB_SLAM2v2/MapGraph.h>
//#include <ORB_SLAM2v2/PoseGraph.h>
//#include <ORB_SLAM2v2/Link.h>
#include <ORB_SLAM2v2/MapSave.h>
#include <ORB_SLAM2v2/MapLoad.h>
#include <ORB_SLAM2v2/MP.h>
#include <ORB_SLAM2v2/KF.h>

using namespace std;

tf::Quaternion hamiltonProduct(tf::Quaternion a, tf::Quaternion b);
tf::Transform GetPoseFromWorld(cv::Mat pose);
geometry_msgs::PoseArray copied_pose_array;

ORB_SLAM2v2::Link link_array;
int link_origin_id;  
int link_destination_id;  
string sSaveFileName;
string sLoadFileName;
string strCurrentDr;

bool GetPoseGraphSrv(ORB_SLAM2v2::MapGraph::Request &req, ORB_SLAM2v2::MapGraph::Response &response){

    
    //test code
    ROS_INFO("callback active");
  
    //get pose array header and pose fields
    response.Data.header = copied_pose_array.header; 
    response.Data.poses = copied_pose_array.poses;
    
    
    //register pose_IDs
    int pose_count = copied_pose_array.poses.size();
    cout << pose_count<<endl;
    for(int i = 0;i<pose_count;i++)
    {
        response.Data.posesId.push_back(i);
        
        //the code below compiles but causes fatal crash
        //the solution must be something similar
        //over flow problem(there is no response.Data.links[pose_count])
        
        /*
        pose[0]                 pose[1]           ...       pose[pose_count-1]
        fromid= null toid=0     fromid=  toid=0             fromid= null toid=0         
        will be brief on 12.3(monday)
        */

        if(i < pose_count){
            link_array.toId = i;
            //response.Data.links[i].toId = i;
            if(i > 0){
               // response.Data.links[i-1].fromId = i-1; 
            link_array.fromId = i-1;
            }
            response.Data.links.push_back(link_array);
            
       }
    }

    return true;
}



bool MapSave(ORB_SLAM2v2::MapSave::Request &req, ORB_SLAM2v2::MapSave::Response &response){

    //test code
    ROS_INFO("MapSave callback");
  
    //get file name
    sSaveFileName = req.file_name;
    cout<<"ros_rgbd::MapSave" << sSaveFileName << endl;
    //get pose array header and pose fields
    response.succeeded = true;

    return true;
}


bool MapLoad(ORB_SLAM2v2::MapLoad::Request &req, ORB_SLAM2v2::MapLoad::Response &response){

    //test code
    ROS_INFO("MapLoad callback");

    //get file name
    sLoadFileName = req.file_name;
    //get pose array header and pose fields
    response.succeeded = true;

    return true;
}

class ImageGrabber
{
public:
    ORB_SLAM2::System* mpSLAM;

    ros::Publisher kf_publisher;
    ros::Publisher kf_stamped_publisher;
    ros::Publisher Odom_pub;

    // /odom    
    ros::Publisher kf_publisher_odom;
    ros::Publisher kf_stamped_publisher_odom;  

    //TF odom->map
    ros::Publisher kf_publisher_om;
    ros::Publisher kf_stamped_publisher_om;   


    tf::TransformBroadcaster* br;

    ros::Publisher poseArrayPub;
    ros::Publisher VisualOdometryPub;

    ros::ServiceServer PoseGraphSrv;
    ros::ServiceServer MapSaveSrv;
    ros::ServiceServer MapLoadSrv;

    bool publish_tf=false;
    bool publish_odom=false;



public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle nh, tf::TransformBroadcaster* _br):mpSLAM(pSLAM){
        kf_publisher = nh.advertise<geometry_msgs::Pose>("/orb_slam/keyframe_optimized", 10);
        kf_stamped_publisher = nh.advertise<geometry_msgs::PoseStamped>("/orb_slam/keyframe_stamped_optimized", 10);
        
        poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("/PoseGraph", 1);
        VisualOdometryPub = nh.advertise<nav_msgs::Odometry>("/VisualOdometry", 1);
        //odom
        kf_publisher_odom = nh.advertise<geometry_msgs::Pose>("/orb_slam_odom/keyframe_optimized_odom", 10);
        kf_stamped_publisher_odom = nh.advertise<geometry_msgs::PoseStamped>("/orb_slam_odom/keyframe_stamped_optimized_odom", 10);
        //TF odom->map
        kf_publisher_om = nh.advertise<geometry_msgs::Pose>("/orb_slam_om/keyframe_optimized_om", 10);
        kf_stamped_publisher_om = nh.advertise<geometry_msgs::PoseStamped>("/orb_slam_om/keyframe_stamped_optimized_om", 10);
       
        //service Pose-graph
        PoseGraphSrv = nh.advertiseService("get_graph", GetPoseGraphSrv);

        //service save/load map
        MapSaveSrv = nh.advertiseService("map_save", MapSave);
        MapLoadSrv = nh.advertiseService("map_load", MapLoad);
        

        br = _br;

        nh.param("publish_tf", publish_tf, publish_tf);
        nh.param("publish_odom", publish_odom, publish_odom);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void ServiceSaveMapCallback();
    void ServiceLoadMapCallback();
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    ros::NodeHandle nh("~");
    ros::NodeHandle n;
    tf::TransformBroadcaster br;


     if(argc != 3)
     {
         cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
         ros::shutdown();
         return 1;
     }

    bool publish_tf = false;
    bool publish_odom = false;
    bool mapping = false;
    bool build_octomap = true;
    string mapBinaryPath = "/map.bin";
    string mapOctomapPath = "/global_octomap.bt";
    string mapPCLPath = "/optimized_pointcloud.pcd";
    int ClientId = 0;
    string homeEnv;

     
    nh.param("publish_tf", publish_tf, publish_tf);
    nh.param("publish_odom", publish_odom, publish_odom);
    nh.param("mapping", mapping, mapping);
    nh.param("build_octomap", build_octomap, build_octomap);


    ORBParams params(publish_tf, publish_odom, mapping, build_octomap);

    mapBinaryPath = strCurrentDr + mapBinaryPath;
    mapOctomapPath = strCurrentDr + mapOctomapPath;
    mapPCLPath = strCurrentDr + mapPCLPath;
    nh.param("mapBinaryPath", mapBinaryPath, mapBinaryPath);
    nh.param("mapOctomapPath", mapOctomapPath, mapOctomapPath);
    nh.param("mapPCLPath", mapPCLPath, mapPCLPath);
    nh.param("mapWorkingPath", strCurrentDr, strCurrentDr);
    nh.param("ClientId", ClientId, ClientId);
    
    params.setMapWorkingPath(strCurrentDr.c_str());
    params.setMapBinaryPath(mapBinaryPath.c_str());
    params.setMapOctomapPath(mapOctomapPath.c_str());
    params.setMapPCLPath(mapPCLPath.c_str());
    params.setClientId(ClientId);
    params.setNodeHandle(n);

    string topic_rgb = "/camera/rgb/image_raw";
    string topic_depth = "/camera/depth/image";


    nh.param<std::string>("topic_rgb", topic_rgb, topic_rgb);
    nh.param<std::string>("topic_depth", topic_depth, topic_depth);


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,params);


    ImageGrabber igb(&SLAM, nh, &br);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, topic_rgb, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, topic_depth, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    string rcv = "CLIENT_MAP" + to_string(ClientId);
    ros::Subscriber rcv_map = n.subscribe(rcv, 1000, &System::ReceiveMapCallback, &SLAM);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

tf::Quaternion hamiltonProduct(tf::Quaternion a, tf::Quaternion b)
{
    tf::Quaternion c;

        c[0] = (a[0]*b[0]) - (a[1]*b[1]) - (a[2]*b[2]) - (a[3]*b[3]);
        c[1] = (a[0]*b[1]) + (a[1]*b[0]) + (a[2]*b[3]) - (a[3]*b[2]);
        c[2] = (a[0]*b[2]) - (a[1]*b[3]) + (a[2]*b[0]) + (a[3]*b[1]);
        c[3] = (a[0]*b[3]) + (a[1]*b[2]) - (a[2]*b[1]) + (a[3]*b[0]);

    return c;
}


void ImageGrabber::ServiceSaveMapCallback(){
    
    if(sSaveFileName.length() > 0){
        cout << "save file "<<endl;
        string strtmp = strCurrentDr;
        strtmp = strtmp +sSaveFileName + ".bin";
        mpSLAM->SaveMap(strtmp);
        
        sSaveFileName = "";
    }

}

void ImageGrabber::ServiceLoadMapCallback(){
    
    if(sLoadFileName.length() > 0){
        cout << "load file "<<endl;
        string strtmp = strCurrentDr; 
        strtmp = strtmp +sLoadFileName + ".bin";
        mpSLAM->RequestServiceLoadMap(strtmp);
        sLoadFileName = "";
    }

}


void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if (pose.empty())    return;
    
    tf::Transform transformCurrent = GetPoseFromWorld(pose);
   
   

    br->sendTransform(tf::StampedTransform(transformCurrent, ros::Time::now(), "map", "base_link"));

    geometry_msgs::Pose kf_pose;
    tf::poseTFToMsg(transformCurrent, kf_pose);
    kf_publisher.publish(kf_pose);

    geometry_msgs::PoseStamped kf_pose_stamped;
    kf_pose_stamped.header.stamp = ros::Time::now();

    kf_pose_stamped.header.frame_id = "map";
    kf_pose_stamped.pose = kf_pose;
    kf_stamped_publisher.publish(kf_pose_stamped);



    /*geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "camera_link";
    br->sendTransform(odom_trans);
*/

    //////////////////////////////////////////////////////
    ///                                                ///
    ///         /odom frame -> camera frame(pose)      ///
    ///         date : 2018.11.26                      ///
    ///                                                ///
    //////////////////////////////////////////////////////

    // odom 
    cv::Mat pose_odom = mpSLAM->TrackPoseOdom();
    //cout << "pose Odom" << poseOdom << endl;
    if (pose_odom.empty())    return;
    tf::Transform transformCurrent_odom = GetPoseFromWorld(pose_odom);

    br->sendTransform(tf::StampedTransform(transformCurrent_odom, ros::Time::now(), "odom", "base_link"));

    //////////////////////////////////////////////////////
    ///                                                ///
    ///         /odom frame -> /map frame              ///
    ///                date : 2018.11.26               ///
    ///                                                ///
    //////////////////////////////////////////////////////

    // odom frame -> map frame 
    
    // variable definition
    // pose from odom frame: pose_odom
    // pose from map frame: pose
    // defined by Tcw  (cv::mat)

    if (pose.empty() || pose_odom.empty())    return;

    //get inverse pose(/map)
    cv::Mat pose_odom_inverse = cv::Mat::eye(4,4,CV_32F);

    cv::Mat Rcw = pose_odom.rowRange(0,3).colRange(0,3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat tcw = pose_odom.rowRange(0,3).col(3);
    cv::Mat Ow = -Rwc*tcw;

    Rwc.copyTo(pose_odom_inverse.rowRange(0,3).colRange(0,3));
    Ow.copyTo(pose_odom_inverse.rowRange(0,3).col(3));
     
    cv::Mat TF_odom_map=  pose_odom_inverse * pose;
    
    tf::Transform transformCurrent_om = GetPoseFromWorld(TF_odom_map);
    if(publish_tf){
          br->sendTransform(tf::StampedTransform(transformCurrent_om, ros::Time::now(), "map", "odom"));
    }
  

    ///////////////////////////////////////////////////////////////
    ////////////////////// Pose-graph               ///////////////
    ///////////////  Date : 2018.11.27      ///////////////////////
    ///////////////////////////////////////////////////////////////

    geometry_msgs::PoseArray aPoseArray;

    vector<cv::Mat> TcwArray = mpSLAM->GetPoseArray();
    
    geometry_msgs::Pose posetmp;

    for(size_t i=0; i<TcwArray.size(); i++)
    {
        tf::Transform tfPoseArray= GetPoseFromWorld(TcwArray[i]);

        geometry_msgs::Pose PoseArrayTmp;
        
        tf::poseTFToMsg(tfPoseArray, PoseArrayTmp);

       aPoseArray.poses.push_back(PoseArrayTmp);
      
       
        
    }

    // define header
    aPoseArray.header.stamp = ros::Time::now();
    aPoseArray.header.frame_id = "/map";
   // mMapGraph.header.stamp = ros::Time::now();
   // mMapGraph.header.frame_id = "/map";
    // Get aPoseArray value

    copied_pose_array = aPoseArray;
    poseArrayPub.publish(aPoseArray);


    //////////////////////////////////////////////////////
    ///                                                ///
    ///         /visual Odometry                      ///
    ///                date : 2018.11.26               ///
    ///                                                ///
    //////////////////////////////////////////////////////

    nav_msgs::Odometry VisualOdometry;

    VisualOdometry.header.stamp = ros::Time::now();
    VisualOdometry.header.frame_id = "/odom";

    VisualOdometry.child_frame_id = "/base_link";

    geometry_msgs::Pose VisualOdometryPoseTmp;
    tf::poseTFToMsg(transformCurrent_odom, VisualOdometryPoseTmp);

    VisualOdometry.pose.pose.position.x = VisualOdometryPoseTmp.position.x;
    VisualOdometry.pose.pose.position.y = VisualOdometryPoseTmp.position.y;
    VisualOdometry.pose.pose.position.z = VisualOdometryPoseTmp.position.z;
    VisualOdometry.pose.pose.orientation.x = VisualOdometryPoseTmp.orientation.x;
    VisualOdometry.pose.pose.orientation.y = VisualOdometryPoseTmp.orientation.y;
    VisualOdometry.pose.pose.orientation.z = VisualOdometryPoseTmp.orientation.z;
    VisualOdometry.pose.pose.orientation.w = VisualOdometryPoseTmp.orientation.w;

    //VisualOdometry.pose.covariance = 0.01;

	if(publish_odom){
    VisualOdometryPub.publish(VisualOdometry);
	}
    ////////////////////////////////////////////////////////


    this->ServiceSaveMapCallback();
    this->ServiceLoadMapCallback();

}


tf::Transform GetPoseFromWorld(cv::Mat pose){
   //Quaternion
    tf::Matrix3x3 tf3d;
    tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
            pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
            pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);
    double aux = tfqt[0];
        tfqt[0]=-tfqt[2];
        tfqt[2]=tfqt[1];
        tfqt[1]=aux;


    //Translation for camera
    tf::Vector3 origin;
    origin.setValue(pose.at<float>(0,3),pose.at<float>(1,3),pose.at<float>(2,3));
    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                            0, 0, 1,
                                            -1, 0, 0);

    tf::Vector3 translationForCamera = origin * rotation270degXZ;

    //Hamilton (Translation for world)
    tf::Quaternion quaternionForHamilton(tfqt[3], tfqt[0], tfqt[1], tfqt[2]);
    tf::Quaternion secondQuaternionForHamilton(tfqt[3], -tfqt[0], -tfqt[1], -tfqt[2]);
    tf::Quaternion translationHamilton(0, translationForCamera[0], translationForCamera[1], translationForCamera[2]);

    tf::Quaternion translationStepQuat;
    translationStepQuat = hamiltonProduct(hamiltonProduct(quaternionForHamilton, translationHamilton), secondQuaternionForHamilton);

    tf::Vector3 translation(translationStepQuat[1], translationStepQuat[2], translationStepQuat[3]);

    //Creates transform and populates it with translation and quaternion
    tf::Transform transformCurrent;
    transformCurrent.setOrigin(translation);
    transformCurrent.setRotation(tfqt);

    return transformCurrent;

}


