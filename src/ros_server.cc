#include <ros/ros.h>
#include "ServerMap.h"
#include "ServerViewer.h"
#include "ORBParams.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "PointCloudTransform.h"

#include <ORB_SLAM2v2/MP.h>
#include <ORB_SLAM2v2/KF.h>
#include "std_msgs/String.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <string>
#include "BoostArchiver.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap/octomap.h>

#define NUMBER_OF_CLIENTS 1
#define INSERT 0
#define ERASE 1
#define UPDATE 2

using boost::asio::ip::tcp;

namespace ORB_SLAM2{

class Communicator{
public:
    Communicator(){
        strSettingsFile = "/home/ros-slam/orb_ws/src/orb_slam/Examples/ROS/ORB_SLAM2/Asus.yaml";
        sm = new ServerMap();
        mpSMapDrawer = new MapDrawer(sm, strSettingsFile);
        mServerViewer = new ServerViewer(mpSMapDrawer, strSettingsFile);
        new thread(&ServerViewer::Run,mServerViewer);
        cout << "Communicator Created" << endl;
    }

    Communicator(char* argv1, char* argv2, ORBParams params){
        sm = new ServerMap();
        voca_path = string(argv1);
        mpSMapDrawer = new MapDrawer(sm, argv2);
        mServerViewer = new ServerViewer(sm, params, mpSMapDrawer, argv2);
        string cmp = "CLIENT_MAP" + to_string(params.getClientId());
        client_map_pub = params.getNodeHandle().advertise<std_msgs::String>(cmp,1000);
        //Test for octomap_rviz
        octomap_rviz = params.getNodeHandle().advertise<octomap_msgs::Octomap>("octomap_rviz",1);
        mapBinaryPath = params.getMapBinaryPath();
        mapOctomapPath = params.getMapOctomapPath();
        clientId = params.getClientId();
        new thread(&ServerViewer::Run,mServerViewer);
        cout << "Communicator Created" << endl;
    }

    void KeyFrameCallback(const ORB_SLAM2v2::KF::ConstPtr& msg){
        if(!sm->ConnectClient)
            return;
        float tcw[16] = {msg->Tcw[0],msg->Tcw[1],msg->Tcw[2],msg->Tcw[3],
        msg->Tcw[4],msg->Tcw[5],msg->Tcw[6],msg->Tcw[7],
        msg->Tcw[8],msg->Tcw[9],msg->Tcw[10],msg->Tcw[11],
        msg->Tcw[12],msg->Tcw[13],msg->Tcw[14],msg->Tcw[15]};
        cv::Mat Tcw(4,4,CV_32F,tcw);
        float twc[16] = {msg->Twc[0],msg->Twc[1],msg->Twc[2],msg->Twc[3],
        msg->Twc[4],msg->Twc[5],msg->Twc[6],msg->Twc[7],
        msg->Twc[8],msg->Twc[9],msg->Twc[10],msg->Twc[11],
        msg->Twc[12],msg->Twc[13],msg->Twc[14],msg->Twc[15]};
        float ow[3] = {msg->Ow[0],msg->Ow[1],msg->Ow[2]};
        cv::Mat Twc(4,4,CV_32F,twc);
        cv::Mat Ow(3,1,CV_32F,ow);
        vector<long unsigned int> cl(begin(msg->CovisibleList), end(msg->CovisibleList));
        vector<long unsigned int> lel(begin(msg->LoopEdgeList), end(msg->LoopEdgeList));
        if(msg->command == INSERT)
            sm->AddKeyFrame(new ServerKeyFrame(msg));
        else if(msg->command == ERASE)
            sm->EraseKeyFrame(msg->mnId);
        else if(msg->command == UPDATE)
            sm->UpdateKeyFrame(new ServerKeyFrame(msg->mnId, Tcw, Twc, Ow, cl, msg->Parent, lel));
    }

    void MapPointCallback(const ORB_SLAM2v2::MP::ConstPtr& msg){
        if(!sm->ConnectClient)
            return;
        if(msg->command == INSERT)
            sm->AddMapPoint(new ServerMapPoint(msg));
        else if(msg->command == ERASE)
            sm->EraseMapPoint(msg->UID);
        else if(msg->command == UPDATE)
            sm->UpdateMapPoint(new ServerMapPoint(msg));
    }

    void KeyFrameData(const ORB_SLAM2v2::KF::ConstPtr& msg){
        if(!sm->ConnectClient)
            return;
        float tcw[16] = {msg->Tcw[0],msg->Tcw[1],msg->Tcw[2],msg->Tcw[3],
        msg->Tcw[4],msg->Tcw[5],msg->Tcw[6],msg->Tcw[7],
        msg->Tcw[8],msg->Tcw[9],msg->Tcw[10],msg->Tcw[11],
        msg->Tcw[12],msg->Tcw[13],msg->Tcw[14],msg->Tcw[15]};
        cv::Mat Tcw(4,4,CV_32F,tcw);
        float twc[16] = {msg->Twc[0],msg->Twc[1],msg->Twc[2],msg->Twc[3],
        msg->Twc[4],msg->Twc[5],msg->Twc[6],msg->Twc[7],
        msg->Twc[8],msg->Twc[9],msg->Twc[10],msg->Twc[11],
        msg->Twc[12],msg->Twc[13],msg->Twc[14],msg->Twc[15]};
        float ow[3] = {msg->Ow[0],msg->Ow[1],msg->Ow[2]};
        cv::Mat Twc(4,4,CV_32F,twc);
        cv::Mat Ow(3,1,CV_32F,ow);
        vector<long unsigned int> cl(begin(msg->CovisibleList), end(msg->CovisibleList));
        vector<long unsigned int> lel(begin(msg->LoopEdgeList), end(msg->LoopEdgeList));
        vector<long unsigned int> mvpMP(begin(msg->mvpMapPoints), end(msg->mvpMapPoints));
        if(msg->command == INSERT){
            sm->AddKeyFrame(new ServerKeyFrame(msg));
        }else if(msg->command == ERASE){
            sm->EraseKeyFrame(msg->mnId);
        }else if(msg->command == UPDATE){
            sm->UpdateKeyFrame(new ServerKeyFrame(msg->mnId, Tcw, Twc, Ow, cl, msg->Parent, lel));
        }
    }

    void MapPointData(const ORB_SLAM2v2::MP::ConstPtr& msg){
        if(!sm->ConnectClient)
            return;
        if(msg->command == INSERT)
            sm->AddMapPoint(new ServerMapPoint(msg));
        else if(msg->command == ERASE)
            sm->EraseMapPoint(msg->UID);
        else if(msg->command == UPDATE)
            sm->UpdateMapPoint(new ServerMapPoint(msg));
    }

    void Shutdown(){

    }

    void SendMap(const std_msgs::String::ConstPtr& msg);
    void CreateOctomap(const std_msgs::String::ConstPtr& msg);

    ServerMap *sm;
    MapDrawer *mpSMapDrawer;
    string strSettingsFile;
    string voca_path;
    ServerViewer *mServerViewer;
    ros::Subscriber kf_sub;
    ros::Subscriber mp_sub;
    ros::Publisher client_map_pub;
    ros::Publisher octomap_rviz;
    const char* mapBinaryPath;
    const char* mapOctomapPath;
    int clientId;
    bool isStop = false;

    int mnId = 0;

};

void Communicator::SendMap(const std_msgs::String::ConstPtr& msg){
    ifstream in(mapBinaryPath, std::ios_base::binary);
    Map *mpMap = new Map();
    if (!in)
    {
        cerr << "Cannot Open Mapfile: " << mapBinaryPath << " , You need create it first!" << std::endl;
        return;
    }

    {
        boost::archive::binary_iarchive ia(in, boost::archive::no_header);
        ia >> mpMap;
    }

    ostringstream sarray;
    {
        boost::archive::binary_oarchive oa(sarray, boost::archive::no_header);
        oa << mpMap;
    }
    
    cout << "Serialized!" << endl;

    std_msgs::String map_msg;
    map_msg.data = sarray.str();
    client_map_pub.publish(map_msg);

    cout << "Done!" << endl;
}

void Communicator::CreateOctomap(const std_msgs::String::ConstPtr& msg){
    ifstream in(mapBinaryPath, std::ios_base::binary);
    Map *mpMap = new Map();
    if (!in)
    {
        cerr << "Cannot Open Mapfile: " << mapBinaryPath << " , You need create it first!" << std::endl;
        return;
    }

    {
        boost::archive::binary_iarchive ia(in, boost::archive::no_header);
        ia >> mpMap;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud;
    vector<MapPoint*> mvpMP = mpMap->GetAllMapPoints();

    for(vector<MapPoint*>::iterator itx = mvpMP.begin(); itx != mvpMP.end(); itx++){
        cv::Mat Tworld = (*itx)->GetWorldPos();
        pcl::PointXYZRGBA p;
        p.x = 5 * Tworld.at<float>(0,0);
        p.y = 5 * Tworld.at<float>(0,1);
        p.z = 5 * Tworld.at<float>(0,2);
        cloud->points.push_back(p);
    }

    PointCloudTransform Pct(cloud);

    transformed_cloud = Pct.transform();

    octomap::OcTree tree( 0.2 );
    for (auto p:transformed_cloud->points)
    {
        // Insert the point of Point Cloud to octomap
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }
    tree.updateInnerOccupancy();
    tree.writeBinary(mapOctomapPath);

    octomap_msgs::Octomap oct_msg;
    octomap_msgs::binaryMapToMsg(tree, oct_msg);
    octomap_rviz.publish(oct_msg);

    cout << "Octomap Created!" << endl;
}

}

using namespace ORB_SLAM2;
using namespace std;

int main(int argc, char *argv[]){
    ros::init(argc, argv, "SERVER");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    ORBParams params;
    int ClientId=2;
    string mapBinaryPath;
    string mapOctomapPath;
    nh.param("ClientId", ClientId, ClientId);
    nh.param("mapBinaryPath", mapBinaryPath, mapBinaryPath);
    nh.param("mapOctomapPath", mapOctomapPath, mapOctomapPath);

    params.setMapBinaryPath(mapBinaryPath.c_str());
    params.setMapOctomapPath(mapOctomapPath.c_str());
    params.setNodeHandle(n);
    params.setClientId(ClientId);

    Communicator ccom(argv[1], argv[2], params);

    string kfName = "KEYFRAME" + to_string(ClientId);
    string mpName = "MAPPOINT" + to_string(ClientId);
    string kfDataName = "KEYFRAME_" + to_string(ClientId);
    string mpDataName = "MAPPOINT_" + to_string(ClientId);
    string cmr = "CREATE_MAP_REQUEST" + to_string(ClientId);
    string cor = "CREATE_OCTOMAP_REQUEST" + to_string(ClientId);

    ros::Subscriber kf_sub = n.subscribe(kfName, 1000, &Communicator::KeyFrameCallback, &ccom);
    ros::Subscriber mp_sub = n.subscribe(mpName, 1000, &Communicator::MapPointCallback, &ccom);
    ros::Subscriber kf_data_sub = n.subscribe(kfDataName, 1000, &Communicator::KeyFrameData, &ccom);
    ros::Subscriber mp_data_sub = n.subscribe(mpDataName, 1000, &Communicator::MapPointData, &ccom);

    ros::Subscriber create_map_request = n.subscribe(cmr, 1000, &Communicator::SendMap, &ccom);
    ros::Subscriber create_octomap_request = n.subscribe(cor, 1000, &Communicator::CreateOctomap, &ccom);

    ros::spin();
    ccom.Shutdown();
    ros::shutdown();

    return 0;
}