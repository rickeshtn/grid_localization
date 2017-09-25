#include"pointcloud_receive.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mrpt_receive");
    pointcloud_receive gl;  //grid_localization
    // ros::AsyncSpinner spinner(0);
    // spinner.start();


    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    ros::waitForShutdown();

    if(gl.SAVE_POINTCLOUD){
        gl.littlePointsMap.saveToPlyFile("/home/guolindong/catkin_ws/src/grid_localization/pointcloud/changshu_g200_01.ply",true);
        ROS_INFO("Point cloud PLY file saved.");
        gl.globalPointsMap.clear();
    }
    if(gl.SAVE_RESULTANDLOG){
        gl.outputFile_raw.close();
        gl.outputFile_result.close();
    }

    return 0;
}
