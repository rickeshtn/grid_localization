#ifndef POINTCLOUD_RECEIVE_H
#define POINTCLOUD_RECEIVE_H

#ifndef Q_MOC_RUN

#include <stdio.h>
#include <time.h>
#include <ros/ros.h>
#include"sensor_msgs/Imu.h"
#include <sensor_msgs/PointCloud2.h>
#include"std_msgs/Float64MultiArray.h"
//#include "point_types.h" //for usage in "pointcloud_recive" to get ring information from "sensor_msgs/pointcloud2"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include"mrpt_bridge/mrpt_bridge.h"
#include"dead_reckoning.h"

#include <mrpt/slam/CGridMapAligner.h>			//map aligner
#include <mrpt/slam/CICP.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
//#include <mrpt/obs/CObservationVelodyneScan.h>  //velodyne Scan
#include <mrpt/opengl/C3DSScene.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CText3D.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CDisk.h> 
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/opengl/COpenGLViewport.h>
#include <mrpt/utils/CFileGZOutputStream.h>//rawlog文件输出
#include <mrpt/utils/CFileOutputStream.h>//文本文件输出
#include <mrpt/utils/CImage.h>
#include <mrpt/system/memory.h>//

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>

#include "ctime"
#include "time.h"

#endif
using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;

using namespace message_filters;



class pointcloud_receive
{
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

    ros::Publisher pose_pub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_velodyne_front = NULL;//(nh_, "front/velodyne_points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_velodyne_rear = NULL;//(nh_, "rear/velodyne_points", 1);
    message_filters::Synchronizer<MySyncPolicy> *sync = NULL;

public:
    int excute_mode;
    int process_counter;

    std::string ini_file_path;
    std::string pointcloud_topic_name;
    std::string load_ply_file_path;
    std::string save_ply_file_path;
    std::string grid_map_path;
    std::string log_path;
    time_t nowtime;
    int step;
    bool notDoingIcpYet, firstTimeShowGps;

    bool SHOW_WINDOW3D;
    bool SHOW_MINIWINDOW;
    bool SHOW_CURRENTLASERPOINTS;
    bool SHOW_POINTCLOUD;
    bool SHOW_POINTCLOUDCOLORED;
    bool SHOW_LASERPOINTS;
    bool SHOW_GRIDMAPLOCAL;
    bool SHOW_GRIDBOXS;
    bool SHOW_ROBOTPOSE;
    bool SHOW_ROBOTPATH;
    bool SHOW_GROUNDTRUTHPATH;
    bool SHOW_GPSEKFPOSE;
    bool USE_GLOBALPOINTCLOUDFORMATCHING;
    bool SAVE_RESULTANDLOG;
    bool SAVE_POINTCLOUD;
    unsigned int SAVE_POINTCLOUDSTEP;
    bool LOAD_POINTCLOUD;
    string NAME_MAPFILE_PLY;
    bool NAME_MAPFILE_IMG;
    bool USE_GPSFORMAPPING;
    bool CAMERA_FOLLOW_ROBOT;
    bool GENERATE_GRIDMAP;
    bool GENERATE_GRIDMAPFILE;
    bool SHOW_GLOBALPOINTCLOUD;

    bool OUTPUT_FILTERED_POSE;

    float output_pose_shift_x, output_pose_shift_y;

    int whichGpsForInitial;
    float timeForGps;
    float gpsInitialStableCounter;

    float minDisBetweenLaserPoints;
    double pointsMap_heightMin;
    double pointsMap_heightMax;
    float pointsMap_clipOutOfRange;
    float pointsMap_clipCenterDistance;

    float gridMap_enhanceStep;
    float gridMap_pzFactor;
    float gridMap_resolution;
    float gridMap_cellPointsThreshold;
    float gridMap_halfSize;

    float pose_front_VLP16_x;
    float pose_front_VLP16_y;
    float pose_front_VLP16_z;
    float pose_front_VLP16_yaw;
    float pose_front_VLP16_pitch;
    float pose_front_VLP16_roll;

    float pose_rear_VLP16_x;
    float pose_rear_VLP16_y;
    float pose_rear_VLP16_z;
    float pose_rear_VLP16_yaw;
    float pose_rear_VLP16_pitch;
    float pose_rear_VLP16_roll;

    int icp_maxIterationFirst;
    int icp_maxIterationAfterFirst;

    CICP icp;
    CICP::TReturnInfo info;
    float runningTime;

    float odoTemp;
    bool icpStarted;//icp是否开始
    bool isMatching;
    int initialGuessStableCounter;

    CTicTac tictac, mainLoop;
    CMatrixDouble33 covariance_matching;

    //array for holding cell properties, for grid map generation
    int **spmLocalGridMap;
    float **spmLocalGridMap_max;
    float **spmLocalGridMap_min;
    int spmGridMap_m;
    int spmGridMap_n;

    //pointsMap and gridMap
    CPoint2D clipCenter, curGridMapCenter; // center point for clipOutOfRange()
    CPoint2D globalGridMapCenter;
    bool gridMapCenterUpdate;
    float dFromPoseToCenter;
    int gridMapHalfSize;
    float centerX;// 42000
    float centerY;// 2700

    //for dead reckoning
    CPose2D poseIncr2D, poseDR2D, poseEst2D, poseEkf2D;
    CPose2D poseDR2D_last, poseEst2D_last, poseEkf2D_last;
    CPose3D poseIncr3D, poseDR3D, poseEst3D, poseEkf3D;
    CPose2D initialGuess;
    CPosePDFGaussian pdfG;
    bool hasCurRobotPoseEst;
    bool isPoseEstGood;
    bool gpsEkfDown;

    //Map相关内容#include <mrpt/utils/CConfigFile.h>
    CSimplePointsMap localPointsMap;//从globalPointsMap中抽取的，用于生成占据栅格地图的局部点云
    CSimplePointsMap globalPointsMap;//从globalPointsMapTemp中抽取的符合高度要求的点云，作为后续生成gridMap的点云
    CSimplePointsMap tempPointsMap, littlePointsMap;
    CColouredPointsMap curPointsMapColored, localPointsMapColored, globalPointsMapColored, tempPointsMapColored, gridMapPointsMap;
    COccupancyGridMap2D localGridMap, globalGridMap;

    //计时
    double timPrev, timNow, timDelta;
    //opengl相关内容
    COpenGLScenePtr scene;                      //主显示场景
    CDisplayWindow3D win3D;                     //窗口
    //COpenGLViewportPtr view_mini;               //小显示场景
    mrpt::opengl::CPointCloudPtr pointCloud, curPointCloud, global_point_cloud;   //单色点云
    mrpt::opengl::CPointCloudColouredPtr pointCloudColored;   //彩色点云
    mrpt::opengl::CGridPlaneXYPtr gridPlane, grid_plane_xy;                  //地平面网格
    mrpt::opengl::CSetOfObjectsPtr objBoxes;                  //box集合
    mrpt::opengl::CBoxPtr objBox;                             //单个box
    mrpt::opengl::CDiskPtr objDisk;                           //小窗口中显示定位结果的点
    mrpt::opengl::CSetOfObjectsPtr objAxis, objGpsEKF;        //定位结果坐标标记、ekf融合结果坐标标记
    mrpt::opengl::CSetOfObjectsPtr objGridMap;                //占据栅格图
    mrpt::opengl::CSetOfObjectsPtr objSetPath;                //路径线段集合
    mrpt::opengl::CSetOfLinesPtr  objEkfPath;                 //单个路径线段
    mrpt::opengl::CSetOfLinesPtr  objMapPath;                 //用于绘制路径
    mrpt::opengl::CTextPtr obj_text_ground_truth, obj_text_estimated_pose;

    CPoint2D miniViewPointTo;

    double poseEstDist2poseEKF; //估计位姿poseEst2D到融合位姿poseEKF的空间距离
    double poseErrorMax;        //估计位姿poseEst2D到融合位姿poseEKF的空间距离 的最大距离，大于此距离则认为定位失败，重新初始化定位过程
    TColorf errorColor;         //颜色

    Lu_Matrix GLOBV_RO; //用于更新glResult_EKF的观测，来自于grid_localization 基于栅格地图匹配的定位结果
    Lu_Matrix X0_init;// = Lu_Matrix(3,1);
    Lu_Matrix P0_init;// = Lu_Matrix(3,3);

    //CObservationVelodyneScanPtr vlpScan = CObservationVelodyneScan::Create();

    //文件输出
    CFileOutputStream outputFile_raw;
    CFileOutputStream outputFile_result;

    //内存占用
    long memUsageByte;
    float memUsageMb;
    float memory_usage_pointcloud;

public:
    dead_reckoning  DR;     //航位推算、ekf融合对象
    pointcloud_receive();
    ~pointcloud_receive(){}

    void grid_localization_init();
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& front,
                             const sensor_msgs::PointCloud2::ConstPtr& rear);
    bool GridMapUpdate();

    bool isGlobalGridMapCenterChange(double robot_x, double robot_y, double Center_x, double Center_y);

    bool myLoadFromBitmapFile(    
        const std::string   &file,
        float           resolution,
        float           xCentralPixel,
        float           yCentralPixel);

    bool myLoadFromBitmap(const mrpt::utils::CImage &imgFl, float resolution, float xCentralPixel, float yCentralPixel);

    bool isNan(float fN);
};

#endif // POINTCLOUD_RECEIVE_H
