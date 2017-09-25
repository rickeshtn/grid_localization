#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

#include"ros/ros.h"
#include"std_msgs/Float64MultiArray.h"
#include"std_msgs/Int8MultiArray.h"
#include"sensor_msgs/Imu.h"
#include"sensor_msgs/NavSatFix.h"

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/obs/CActionRobotMovement2D.h>

#include"pointToGeo.h"
#include"ZHU_EKF.h"
#include"Lu_Matrix.h"
#include "can_msgs/SpeedMilSteer.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::obs;

#define Pi 3.1415926536
#define  cal_factor 0.0210386
#define gyro_z_offset 0.0026

#define latMean 31//31.027
#define lonMean 121//121.435

#define pulse2meter_radio cal_factor

class dead_reckoning
{
    ros::NodeHandle nh_;
    ros::Subscriber velocity_sub_;//用来订阅消息
    ros::Subscriber pulse_sub;
    ros::Subscriber imu_sub_;
    ros::Subscriber rtkgps_sub_;
    ros::Subscriber garmin_sub;
    ros::Subscriber n280_sub;
    ros::Subscriber n280Orientation_sub;
    std::string gps_topic_name;
public:
    std_msgs::Float64MultiArray velocity_data;
    double velocity;
    std_msgs::Float64MultiArray pulse_data;
    sensor_msgs::Imu imu_data;
    sensor_msgs::NavSatFix gps_data;

    //GPS related values
    pointToGeo gpsOrigin;
    pointToGeo gpsPos, gpsZero, gpsOut;
    double latmean,lonmean,scale;
    float n280Orientation;

    //EKF related values  //using ZHU_EKF & Lu_Matrix
    ZHU_EKF GPSINS_EKF; //gps的滤波结果
    ZHU_EKF glResult_EKF;//定位结果的滤波结果
    Lu_Matrix GPSOBV_RO, X0, P0;
    double XSENS_Diff;
    double speedCov;
    double steerCov;
    double speed_time;
    double speed;
    double mil;
    double angularVelocity;
    double rtkGps_x, rtkGps_y;
    double firstGps_x, firstGps_y;
    double gpsSum_x, gpsSum_y;
    double lat, lon;
    double uTcTime;
    bool gpsFirstGet, gpsCanGetSecond;
    int initGpsCounter;
    bool ekf_result_ready;
    bool firstMovement, firstGpsUpdate;
    bool odoError;

    int whichGpsForInitial;

    //Others
    CPose2D robot_pose_inc, robot_pose_gps, robot_poseGps_ekf, robot_poseResult_ekf;
    CPose2D poseIncr2D, poseDR2D, poseGps2D, poseGps2D_last;
    CPose3D poseIncr3D, poseDR3D, poseGps3D;
    CPose2D poseEst;
    CPosePDFPtr poseMatchPdf;
    CTicTac tictacDR;
    double ticPre, ticNow, ticDelta;
    double timePresent, timePrevious;
    bool firstTime;
    bool notDoingIcpYet;

    double yaw, pitch, roll, yawVel;
    float pulesSum;

    float yaw_angle;
    float roll_angle;
    float pitch_angle;
    float yaw_angle_veolcity_sum;
    float yaw_velocity_last;
    float pulse_sum;
    float mileage_sum;
    float mileage_last, mileage_present, mileage_incr;
    dead_reckoning();
    ~dead_reckoning(){}

    void xsens_callback(const sensor_msgs::Imu &msg);
    void odo_callback(const can_msgs::SpeedMilSteer &msg  );
    void rtk_callback(const sensor_msgs::NavSatFix &msg );
    // void garmin_callback(const sensor_msgs::NavSatFix &msg );
    //void n280Orientation_callback(const std_msgs::Float64 &msg );

    void xy2latlon(double x, double y, double &lat, double &lon);
    void calculate_pose_inc();

    void get_poseGps();     //only gps (raw gps data)
    void get_poseGps_ekf();  //gps + odometry + imu
    void get_poseResult_ekf(); //using ekf to filter the localization result
    void clear_sum();
};

#endif // DEAD_RECKONING_H
