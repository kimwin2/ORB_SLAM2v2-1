#include "ServerViewer.h"
#include "std_msgs/String.h"

namespace ORB_SLAM2
{

ServerViewer::ServerViewer(MapDrawer *pSMapDrawer, const string &strSettingPath) : mpSMapDrawer(pSMapDrawer), bConnect(true)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if (fps < 1)
        fps = 30;
    mT = 1e3 / fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if (mImageWidth < 1 || mImageHeight < 1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

ServerViewer::ServerViewer(ServerMap *pSMap, ORBParams params, MapDrawer *pSMapDrawer, const string &strSettingPath) : mpSMapDrawer(pSMapDrawer), mpSMap(pSMap), bConnect(true)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if (fps < 1)
        fps = 30;
    mT = 1e3 / fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if (mImageWidth < 1 || mImageHeight < 1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];

    clientId = params.getClientId();
    mapBinaryPath = params.getMapBinaryPath();
    string cmr = "CREATE_MAP_REQUEST" + to_string(clientId);
    map_pub = params.getNodeHandle().advertise<std_msgs::String>(cmr, 1000);
}

void ServerViewer::Run()
{
    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer", 1024, 768);
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
    pangolin::Var<bool> menuConnect("menu.Connect", true, true);
    pangolin::Var<bool> menuReset("menu.Reset", false, false);
    pangolin::Var<bool> menuSave("menu.Save", false, false);
    pangolin::Var<bool> menuSend("menu.Send", false, false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while (1)
    {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        mpSMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        mpSMapDrawer->DrawServerMapPoints();
        mpSMapDrawer->DrawServerKeyFrames();

        pangolin::FinishFrame();

        if (menuConnect && !bConnect)
        {
            mpSMap->ConnectToClient();
            bConnect = true;
        }
        else if (!menuConnect && bConnect)
        {
            mpSMap->DisconnectToClient();
            bConnect = false;
        }

        if (menuReset)
        {
            mpSMap->Clear();
            menuReset = false;
        }

        if (menuSave)
        {
            ofstream out(mapBinaryPath, std::ios_base::binary);

            cout << "Creating new map" << endl;
            Map *mpMap = new Map();
            map<unsigned int, MapPoint *> mspMapPoints;
            map<unsigned int, KeyFrame *> mspKeyFrames;
            map<unsigned int, ServerMapPoint *> mspSMP = mpSMap->GetAllMapPoints();
            map<unsigned int, ServerKeyFrame *> mspSKF = mpSMap->GetAllKeyFrames();
            for (map<unsigned int, ServerKeyFrame *>::iterator itx = mspSKF.begin(); itx != mspSKF.end(); itx++)
            {
                KeyFrame *pKF = new KeyFrame(itx->second, mpMap);
                if (pKF == NULL)
                    continue;
                mspKeyFrames.insert({itx->first, pKF});
            }
            for (map<unsigned int, ServerMapPoint *>::iterator itx = mspSMP.begin(); itx != mspSMP.end(); itx++)
            {
                MapPoint *pMP = new MapPoint(itx->second, mpMap);
                map<unsigned int, unsigned int> mObs = itx->second->mObservations;
                for (map<unsigned int, unsigned int>::iterator itor = mObs.begin(); itor != mObs.end(); itor++)
                {
                    KeyFrame *pKF = mspKeyFrames[itor->first];
                    if (pKF)
                    {
                        pMP->AddObservation(pKF, itor->second);
                        pKF->AddMapPoint(pMP, itor->second);
                        mspMapPoints.insert({itx->first, pMP});
                    }
                }
                pMP->SetObservation();
            }
            cout << "mspMapPoints.insert" << endl;
            for (map<unsigned int, ServerKeyFrame *>::iterator itx = mspSKF.begin(); itx != mspSKF.end(); itx++)
            {
                KeyFrame *pKF = mspKeyFrames[itx->first];
                if (itx->second->parentId != -1)
                {
                    KeyFrame *parentKF = mspKeyFrames[itx->second->parentId];
                    if (parentKF)
                    {
                        pKF->ChangeParent(parentKF);
                        parentKF->AddChild(pKF);
                    }
                }
            }

            mpMap->AddKeyFrame(mspKeyFrames);
            mpMap->AddMapPoint(mspMapPoints);
            mpMap->mvpKeyFrameOrigins.push_back(mspKeyFrames[mpSMap->GetKeyFrameOrigin()]);

            cout << "Strat to serialize" << endl;;
            {
                boost::archive::binary_oarchive oa(out, boost::archive::no_header);
                oa << mpMap;
            }

            cout << "Serialized!" << endl;
            menuSave = false;
        }

        if (menuSend)
        {
            std_msgs::String msg;
            msg.data = "Hello?" + clientId;
            cout << msg.data << endl;
            map_pub.publish(msg);
            menuSend = false;
        }
    }
}

} // namespace ORB_SLAM2