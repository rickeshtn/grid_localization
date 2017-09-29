#include "pointcloud_receive.h"


pointcloud_receive::pointcloud_receive()
{
    step = 0;   //计数器，全局
    DR.whichGpsForInitial = whichGpsForInitial; //选择用于初始化的GPS，低精度和高精度两种
    notDoingIcpYet = true;
    DR.notDoingIcpYet = true;   //是否已开始ICP匹配
    firstTimeShowGps = true;    //
    odoTemp = 0;
    icpStarted = false;
    centerX = -21700;//42500;//42000
    centerY = 66100;//2700;//2700
    CPoint2D gridPlaneCenter = CPoint2D(centerX, centerY);//地面网格中心
    poseEstDist2poseEKF = 0;
    gpsEkfDown = false;

    GLOBV_RO = Lu_Matrix(2,2);//
    X0_init = Lu_Matrix(3,1);
    P0_init = Lu_Matrix(3,3);
    X0_init(0)=0;
    X0_init(1)=0;
    X0_init(2)=0;
    P0_init(0,0)=1;
    P0_init(1,1)=1;
    P0_init(2,2)=0.1;

    win3D.setWindowTitle("CyberFly_View");
    win3D.resize(1280, 720);
    win3D.setCameraAzimuthDeg(270);//方向沿y轴由负方向向正方向看
    win3D.setCameraElevationDeg(90);//俯角20°
    win3D.setCameraPointingToPoint(0, 0, 0);//指向(0,10,0)点
    win3D.setCameraZoom(150);

    //载入config.ini中的设置内容
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");			//定义私有节点句柄，用于传递参数

    sub_velodyne_front = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, "/front/velodyne_points", 2);
    sub_velodyne_rear = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, "/rear/velodyne_points", 2);
    sync = new Synchronizer<MySyncPolicy> (MySyncPolicy(10), *sub_velodyne_front, *sub_velodyne_rear);
    sync->registerCallback(boost::bind(&pointcloud_receive::pointcloud_callback, this, _1, _2));


    pose_pub = nh.advertise<std_msgs::Float64MultiArray>("pose_vlp16",2);

    pnh.param<int>("excute_mode", excute_mode, 2);
    pnh.getParam("excute_mode",excute_mode);
    grid_localization_init(); //初始化部分要在选择模式后执行，因为初始化函数中要根据执行模式加载不同的配置文件
    

    initialGuessStableCounter = 0;
    initialGuess = CPose2D(0,0,0);
    hasCurRobotPoseEst = false;
    isPoseEstGood = true;

    globalPointsMap.enableFilterByHeight(true);
    globalPointsMap.setHeightFilterLevels(pointsMap_heightMin, pointsMap_heightMax);
    globalPointsMap.insertionOptions.minDistBetweenLaserPoints = minDisBetweenLaserPoints;
    localPointsMap.enableFilterByHeight(true);
    localPointsMap.setHeightFilterLevels(pointsMap_heightMin, pointsMap_heightMax);
    localPointsMap.insertionOptions.minDistBetweenLaserPoints = minDisBetweenLaserPoints;

    curPointsMapColored.enableFilterByHeight(true);
    curPointsMapColored.setHeightFilterLevels(pointsMap_heightMin,pointsMap_heightMax);
    curPointsMapColored.insertionOptions.minDistBetweenLaserPoints = minDisBetweenLaserPoints;
    localPointsMapColored.enableFilterByHeight(true);
    localPointsMapColored.setHeightFilterLevels(pointsMap_heightMin,pointsMap_heightMax);
    localPointsMapColored.insertionOptions.minDistBetweenLaserPoints = minDisBetweenLaserPoints;
    globalPointsMapColored.enableFilterByHeight(true);
    globalPointsMapColored.setHeightFilterLevels(pointsMap_heightMin,pointsMap_heightMax);
    globalPointsMapColored.insertionOptions.minDistBetweenLaserPoints = minDisBetweenLaserPoints;

    timPrev = ros::Time::now().toSec();
    poseDR2D = CPose2D(0,0,0);
    poseDR3D = CPose3D(0,0,0);
    poseEst2D = CPose2D(-21729,66191,-0.9);;

    gridPlane = CGridPlaneXY::Create(-1000+centerX,1000+centerX,-1000+centerY,1000+centerY,0.01,10,0.5);
    gridPlane->setColor(0.375,0.375,0.375);
    objAxis = opengl::stock_objects::CornerXYZSimple(2,2);
    objAxis->setName("poseEst");
    objAxis->enableShowName(true);
    objGpsEKF = opengl::stock_objects::CornerXYZSimple(2,2);
    objGpsEKF->setName("GT");
    objGpsEKF->enableShowName(true);
    //objGpsEKF->setColor(0,0,1);

    obj_text_ground_truth = mrpt::opengl::CText::Create();
    obj_text_estimated_pose;

    objEkfPath = CSetOfLines::Create();
    objSetPath = CSetOfObjects::Create();

    objDisk = CDisk::Create();
    objDisk->setDiskRadius(30 );
    objDisk->setColor(1,0.2,1);


    float maxx = -100000000;
    float maxy = -100000000;
    float minx = 100000000;
    float miny = 100000000;

//    objMapPath = CSetOfLines::Create();
//    objMapPath->setColor(0.9,0.8,1);
//    objMapPath->setLineWidth(2.5);
//    miniViewPointTo = CPoint2D((maxx+minx)/2,(maxy+miny)/2);

    clipCenter = CPoint2D(0,0);
    curGridMapCenter = CPoint2D(0,0);
    dFromPoseToCenter = 20;

    CTicTac  timeTest;
    timeTest.Tic();
    //空间分配
    spmGridMap_m = gridMap_halfSize*2 / gridMap_resolution;
    spmGridMap_n = gridMap_halfSize*2 / gridMap_resolution;
    spmLocalGridMap = (int**)malloc(sizeof(int*)* spmGridMap_m); //¿ª±ÙÐÐ
    spmLocalGridMap_max = (float**)malloc(sizeof(float*)* spmGridMap_m);
    spmLocalGridMap_min = (float**)malloc(sizeof(float*)* spmGridMap_m);

    //initial the gridMap correlate counting array
    for (int i = 0; i < spmGridMap_m; i++){
        *(spmLocalGridMap + i) = (int*)malloc(sizeof(int)* spmGridMap_n);//¿ª±ÙÁÐ
        *(spmLocalGridMap_max + i) = (float*)malloc(sizeof(float)* spmGridMap_n);
        *(spmLocalGridMap_min + i) = (float*)malloc(sizeof(float)* spmGridMap_n);
    }
    //erase the gridMap correlate counting array
    for (int i = 0; i < spmGridMap_m; i++){
        for (int j = 0; j < spmGridMap_n; j++){
            spmLocalGridMap[i][j] = 0;
            spmLocalGridMap_max[i][j] = 0;
            spmLocalGridMap_min[i][j] = 0;
        }
    }

    ROS_INFO("time usage: %.3f ms",timeTest.Tac()*1000);

    memory_usage_pointcloud = 0;

    //gridMap initialization
    if(LOAD_POINTCLOUD){
        //load point cloud from ply file
        //globalPointsMap_02.ply是３月底演示之前采集的地图，用的ｒｔｋ的单点模式
        tictac.Tic();
        printf("Loading globalPointsMap...\n");
        //globalPointsMap.loadFromPlyFile(NAME_MAPFILE_PLY);//globalPointsMap_02.ply
        CSimplePointsMap globalPointsMapTemp;//获取globalPointsMap前的过度，根据高度配置抽取globalPointsMap
        globalPointsMapTemp.loadFromPlyFile("/home/guolindong/catkin_ws/src/grid_localization/pointcloud/changshu_g200_every_2_whole_1.ply");
        float max_x, max_y, max_z, min_x, min_y, min_z;
        globalPointsMapTemp.boundingBox(min_x, max_x, min_y, max_y, min_z, max_z);
        globalPointsMapTemp.extractPoints(
            TPoint3D(min_x, min_y, pointsMap_heightMin), 
            TPoint3D(max_x, max_y, pointsMap_heightMax), 
            &globalPointsMap);

        printf("globalPointsMap load in %.3f s.\nglobalPointsMap size: %i points\n", tictac.Tac(), (int)globalPointsMap.size());
        ROS_INFO("ply file path is %s",load_ply_file_path.c_str());

        memory_usage_pointcloud = (float)getMemoryUsage()/(1024*1024);

    }//end of ply file loading 


    if(SAVE_RESULTANDLOG){
        outputFile_result.open("/home/guolindong/catkin_ws/src/grid_localization/save_log/changshu_20170809_01.txt",0);
    }

    tictac.Tic();//start stop watch

    ROS_INFO("Grid_localization initialization done...");
    ROS_INFO("memory usage after initialization: %fMb",memory_usage_pointcloud);
}// end of initialization

//Main process, call back when receive pointCloud2 (from vlp-16)
//void pointcloud_receive::pointcloud_callback(const sensor_msgs::PointCloud2 & msg)
void pointcloud_receive::pointcloud_callback(
        const sensor_msgs::PointCloud2::ConstPtr& front,
        const sensor_msgs::PointCloud2::ConstPtr& rear)
{
    mainLoop.Tic();//start the stop watch
    try{
        CColouredPointsMap curPointsMapColoredTemp,curPointsMapColoredFront,curPointsMapColoredRear;
        mrpt_bridge::copy2colouredPointsMap(front,curPointsMapColoredFront); //to CColoredPointsMap
        mrpt_bridge::copy2colouredPointsMap(rear,curPointsMapColoredRear);

        curPointsMapColoredFront.changeCoordinatesReference(
            CPose3D(pose_front_VLP16_x,
                    pose_front_VLP16_y,
                    pose_front_VLP16_z,
                    pose_front_VLP16_yaw,
                    pose_front_VLP16_pitch,
                    pose_front_VLP16_roll));

        curPointsMapColoredRear.changeCoordinatesReference(
            CPose3D(pose_rear_VLP16_x,
                    pose_rear_VLP16_y,
                    pose_rear_VLP16_z,
                    pose_rear_VLP16_yaw,
                    pose_rear_VLP16_pitch,
                    pose_rear_VLP16_roll));

        curPointsMapColoredTemp.insertionOptions.minDistBetweenLaserPoints = minDisBetweenLaserPoints;
        curPointsMapColoredTemp.insertAnotherMap(&curPointsMapColoredFront,CPose3D(0,0,0,0,0,0));
        curPointsMapColoredTemp.insertAnotherMap(&curPointsMapColoredRear,CPose3D(0,0,0,0,0,0));

        float max_x, max_y, max_z, min_x, min_y, min_z;
        curPointsMapColoredTemp.boundingBox(min_x, max_x, min_y, max_y, min_z, max_z);
        curPointsMapColoredTemp.extractPoints(
            TPoint3D(min_x, min_y, pointsMap_heightMin), 
            TPoint3D(max_x, max_y, pointsMap_heightMax), 
            &curPointsMapColored,
            1,0.1,0.1);

    }
    catch (ros::Exception &e){
        ROS_ERROR("pointcloud_callback exception: %s", e.what());
        return;
    }

    //if don't use gps for mapping and localization
    if(isPoseEstGood)
    {
        if (DR.firstGpsUpdate)
        {
            DR.calculate_pose_inc();    //for ordinary usage, like localization
            DR.get_poseGps_ekf();//用这个结果与glResult进行对比，假设poseGps_ekf为groundTruth   //此处该函数还包含对glResult进行滤波
            //DR.get_pose_ekf();
            poseIncr2D = CPose2D(DR.robot_pose_inc.x(),DR.robot_pose_inc.y(),DR.robot_pose_inc.phi());
            poseIncr3D = CPose3D(DR.robot_pose_inc.x(),DR.robot_pose_inc.y(),0,DR.robot_pose_inc.phi(),0,0);

            poseDR2D += poseIncr2D;
            poseDR3D += poseIncr3D;

            //if need relocate using ICP according to the current GPS(or if ICP failed) 
            CPoint2D pointEst2D = CPoint2D(poseEst2D.x(),poseEst2D.y());
            poseEkf2D = CPose2D(DR.robot_poseGps_ekf.x(),DR.robot_poseGps_ekf.y(),DR.robot_poseGps_ekf.phi());
            if (isNan(poseEkf2D.x()) || isNan(poseEkf2D.y() ))
            {
                X0_init(0) = poseEkf2D_last.x();
                X0_init(1) = poseEkf2D_last.y();
                X0_init(2) = poseEkf2D_last.phi();
                DR.GPSINS_EKF.init(3,X0_init,P0_init);//---------------------------NaN
            }
            if (isNan(poseEst2D.x()) || isNan(poseEst2D.y() ))
            {
                X0_init(0) = poseEkf2D_last.x();
                X0_init(1) = poseEkf2D_last.y();
                X0_init(2) = poseEkf2D_last.phi();
                DR.glResult_EKF.init(3,X0_init,P0_init);//---------------------------NaN
            } 
            //poseEstDist2poseEKF = pointEst2D.distance2DTo(poseEkf2D.x(),poseEkf2D.y());
            if( poseEstDist2poseEKF > poseErrorMax)
            {
                isPoseEstGood = false;
                notDoingIcpYet = true; 
                DR.notDoingIcpYet = true;
                initialGuessStableCounter = 0;
            }

            //if (LOAD_POINTCLOUD)
            {
                poseEst2D_last = poseEst2D;
                poseEkf2D_last = poseEkf2D;
            }
            //else if(SAVE_POINTCLOUD) poseEst2D_last = poseEkf2D;
            if(SAVE_POINTCLOUD) poseEst2D_last = poseEkf2D;
        }
    }
    else{
        DR.get_poseGps_ekf();//initialGuessStableCounter if use RTK-GPS to generate map
        poseEkf2D = CPose2D(DR.robot_poseGps_ekf.x(),
                            DR.robot_poseGps_ekf.y(),
                            DR.robot_poseGps_ekf.phi());
        if (isNan(poseEkf2D.x()) || isNan(poseEkf2D.y()) )
        {
            X0_init(0) = poseEkf2D_last.x();
            X0_init(1) = poseEkf2D_last.y();
            X0_init(2) = poseEkf2D_last.phi();           
            DR.GPSINS_EKF.init(3,X0_init,P0_init);//--------------------------------NaN
        }
        if (isNan(poseEst2D.x()) || isNan(poseEst2D.y() ))
        {
            X0_init(0) = poseEkf2D_last.x();
            X0_init(1) = poseEkf2D_last.y();
            X0_init(2) = poseEkf2D_last.phi();   
            DR.glResult_EKF.init(3,X0_init,P0_init);//---------------------------NaN
        }
        poseEst2D = poseEkf2D;
        CPoint2D pointEst2D = CPoint2D(poseEst2D.x(),poseEst2D.y());
        //poseEstDist2poseEKF = pointEst2D.distance2DTo(poseEkf2D.x(),poseEkf2D.y());

        //for matching using history pointclouds
        poseIncr2D = CPose2D(DR.robot_pose_inc.x(),
                             DR.robot_pose_inc.y(),
                             DR.robot_pose_inc.phi());
        poseIncr3D = CPose3D(DR.robot_pose_inc.x(),
                             DR.robot_pose_inc.y(),
                             0,
                             DR.robot_pose_inc.phi(),
                             0,
                             0);
        poseDR2D += poseIncr2D;
        poseDR3D += poseIncr3D;

        initialGuessStableCounter++;//跟随一段时间
        if(initialGuessStableCounter > gpsInitialStableCounter)
        {
            icpStarted = true;
            isPoseEstGood = true;
        }
        else icpStarted = false;//waiting for a stable gps used for ICP initialguess

        poseEst2D_last = poseEst2D;
        poseEkf2D_last = poseEkf2D;  
    }// end of else yaw_angle_veolcity_sum

    if(!USE_GLOBALPOINTCLOUDFORMATCHING)
    {
        if(SAVE_POINTCLOUD)
        {
            float tt = tictac.Tac();
            if((tt>timeForGps) && (step % SAVE_POINTCLOUDSTEP == 0) && (DR.velocity!=0))
            {
                globalPointsMap.insertAnotherMap(
                        &curPointsMapColored,
                        CPose3D(poseEkf2D.x(),poseEkf2D.y(),0,poseEkf2D.phi(),0,0)
                );//insert only when velocity is not zero
            }
        }
        step++;
    }

    if(isGlobalGridMapCenterChange(poseEst2D.x(),
                                   poseEst2D.y(),
                                   curGridMapCenter.x(),
                                   curGridMapCenter.y()))
    {
        char tempFileName[100];
        //文件夹路径
        std::string grid_map_file_path="/home/guolindong/catkin_ws/src/grid_localization/map/changshu_g200_every2_0.1";
        //具体的地图图片文件名
        grid_map_file_path.append("/%i_%i.png");
        sprintf(tempFileName,grid_map_file_path.c_str(),(int)curGridMapCenter.x(), (int)curGridMapCenter.y());
        //若能够读取到地图
        if(myLoadFromBitmapFile(tempFileName,
                                gridMap_resolution,
                                gridMap_halfSize-curGridMapCenter.x()/gridMap_resolution,
                                gridMap_halfSize-curGridMapCenter.y()/gridMap_resolution))
        {
            ROS_INFO("Map loaded: %f, %f", gridMap_halfSize - curGridMapCenter.x(),
                     gridMap_halfSize - curGridMapCenter.y());
        }
        //若读不到地图,考虑是否根据点云生成占据栅格地图
        else if(GENERATE_GRIDMAP && LOAD_POINTCLOUD)
        {
            CTicTac gridTic;
            gridTic.Tic();
            localPointsMap.clear();
            globalPointsMap.extractPoints(
                CPoint3D(curGridMapCenter.x() - gridMap_halfSize + gridMap_resolution*2,
                         curGridMapCenter.y() - gridMap_halfSize + gridMap_resolution*2,
                         pointsMap_heightMin),
                CPoint3D(curGridMapCenter.x() + gridMap_halfSize - gridMap_resolution*2,
                         curGridMapCenter.y() + gridMap_halfSize - gridMap_resolution*2,
                         pointsMap_heightMax),
                &localPointsMap);

            float xmax = curGridMapCenter.x() + gridMap_halfSize;
            float xmin = curGridMapCenter.x() - gridMap_halfSize;
            float ymax = curGridMapCenter.y() + gridMap_halfSize;
            float ymin = curGridMapCenter.y() - gridMap_halfSize;

            //if max & min calculation error
            if(!((xmax>xmin)&&(ymax>ymin)))
            {
                ROS_INFO("setSize error, xmax %f, xmin %f, ymax %f, ymin %f, posex %f, posey %f",
                         xmax,xmin,ymax,ymin,poseEst2D.x(),poseEst2D.y());
            }
            else{
                localGridMap.setSize(xmin,xmax,ymin,ymax, gridMap_resolution,1);
                for (int i = 0; i < spmGridMap_m; i++)
                {
                    for (int j = 0; j < spmGridMap_n; j++)
                    {
                        spmLocalGridMap[i][j] = 0;
                        spmLocalGridMap_max[i][j] = 0;
                        spmLocalGridMap_min[i][j] = 0;
                    }
                }
               
                std::vector<float> tx, ty, tz;
                localPointsMap.getAllPoints(tx, ty, tz);
                int lpmSize = tx.size();   //local points map size(points number)
                int xx, yy;
                for (int i = 0; i < lpmSize; i++)
                {
                    xx = localGridMap.x2idx(tx[i]);
                    yy = localGridMap.y2idx(ty[i]);
                    //localGridMap.setCell(xx,yy,0);
                    /******************************/
                    spmLocalGridMap[xx][yy] += 1;
                    if(tz[i]>spmLocalGridMap_max[xx][yy]){spmLocalGridMap_max[xx][yy] = tz[i];} //===
                    else if(tz[i]<spmLocalGridMap_min[xx][yy]){spmLocalGridMap_min[xx][yy] = tz[i];} //===
                    /******************************/
                }
                /******************************/
                float gap, density, tempValue;
                float cx, cy;
                int i, j;
                for (i = 0; i < spmGridMap_m; i++)
                {
                    for (j = 0; j < spmGridMap_n; j++)
                    {
                        //first, counting the points amount in the cell
                        if (spmLocalGridMap[i][j] < gridMap_cellPointsThreshold) continue;
                        gap = spmLocalGridMap_max[i][j];//-spmLocalGridMap_min[i][j];
                        if(!gap) continue;
                        else density = (float)spmLocalGridMap[i][j];//density here more like quantity
                        tempValue = 1 - gap*gridMap_pzFactor-density*gridMap_enhanceStep;//i feel sorry for this
                        if(tempValue<0) localGridMap.setCell(i,j,0);
                        else localGridMap.setCell(i,j,tempValue);
                    }//end of loop j
                }//end of loop i
                //保存生成的gridmap图片文件
                if(GENERATE_GRIDMAPFILE){
                    char occupancyMapFileName[100];
                    std::string grid_map_file_path_temp=
                            "/home/guolindong/catkin_ws/src/grid_localization/map/changshu_g200_every2_0.1";
                    grid_map_file_path_temp.append("/%i_%i.png");
                    sprintf(occupancyMapFileName,
                            grid_map_file_path_temp.c_str(),
                            (int)curGridMapCenter.x(),
                            (int)curGridMapCenter.y());
                    localGridMap.saveAsBitmapFile(occupancyMapFileName);
                    ROS_INFO("grid map file saved");
                }
            }
            ROS_INFO("grid map generated in %.3fms",gridTic.Tac()*1000);
        }//end of calculate grid map local
        //若既没有读到地图，又不根据globalPointsMap生成地图
        else {
            //根据局部点云数据生成地图？
        }
    }//end of else


    //-----------------------------------Map Aligner-----------------------------------------
    //if (LOAD_POINTCLOUD && icpStarted && !USE_GPSFORMAPPING)
    if (icpStarted && !USE_GPSFORMAPPING)
    {
        //定位初始化阶段，若还未进行过icp，则利用GPS结果进行初始假设
        if(notDoingIcpYet)
        {
            initialGuess = poseEkf2D;//初始化阶段利用EKF(gps+odo+imu)融合位姿估计
            notDoingIcpYet = false; 
            DR.notDoingIcpYet = false;
//            icp.options.thresholdDist = 0.3;

            //DR.glResult_EKF初始化阶段，即进入grid_localizaiton的地图匹配阶段后才进行初始化
            Lu_Matrix X0 = Lu_Matrix(3,1);
            Lu_Matrix P0 = Lu_Matrix(3,3);
            X0(0)=poseEkf2D.x();
            X0(1)=poseEkf2D.y();
            X0(2)=poseEkf2D.phi();
            if(X0(2)<0)X0(2)+=2*PI;
            P0(0,0)=1,P0(1,1)=1,P0(2,2)=0.1;
            DR.glResult_EKF.init(3,X0,P0);//grid_localization结果的EKF的 初始化
        }
        //若icp已经获得初始定位结果（但通常是比较糟糕的结果）
        else{
            icp.options.maxIterations = icp_maxIterationAfterFirst;
            icp.options.thresholdDist = 1;
            initialGuess = poseEst2D_last + poseIncr2D;// + rando;
        }
        //以上判断最终会给出用于icp的初始假设
        pdfG = CPosePDFGaussian(initialGuess);//

        //若用占据栅格地图作为参考地图进行匹配定位
        if(!USE_GLOBALPOINTCLOUDFORMATCHING)
        {
            CPosePDFPtr pdf = icp.AlignPDF(
                &localGridMap,			// Reference map //localPointsMap
                &curPointsMapColored,	// Map to be aligned
                pdfG,                   // initial estimation
                &runningTime, 
                (void*)&info);				
            poseEst2D = pdf->getMeanVal();
            pdf->getCovarianceAndMean(covariance_matching, poseEst2D);
            hasCurRobotPoseEst = true;
            if(! (step%50) )
            {
                ROS_INFO("cov00: %.6f, cov11: %.6f, cov22: %.6f, gd: %.3f,",
                    (float)covariance_matching(0,0),
                    (float)covariance_matching(1,1),
                    (float)covariance_matching(2,2),
                    info.goodness);
            }
        }

        //若用全局点云地图作为参考进行匹配定位
        else
        {
            if(!LOAD_POINTCLOUD)
                ROS_INFO("ERROR, use global point map for matching need LOAD_POINTCLOUD=true!");
            localPointsMap.clear();
            globalPointsMap.extractPoints(
                    CPoint3D(curGridMapCenter.x() - gridMap_halfSize+0.2,
                             curGridMapCenter.y() - gridMap_halfSize+0.2,
                             pointsMap_heightMin),
                    CPoint3D(curGridMapCenter.x() + gridMap_halfSize-0.2,
                             curGridMapCenter.y() + gridMap_halfSize-0.2,
                             pointsMap_heightMax),
                    &localPointsMap);

            CPosePDFPtr pdf = icp.AlignPDF(
                &localPointsMap,			// Reference map
                &curPointsMapColored,		// Map to be aligned
                pdfG);				// Starting estimate
            poseEst2D = pdf->getMeanVal();
            hasCurRobotPoseEst = true;
        }

        GLOBV_RO(0,0) = sqrt((float)covariance_matching(0,0));
        GLOBV_RO(1,1) = sqrt((float)covariance_matching(1,1));
        GLOBV_RO*=GLOBV_RO;
        DR.glResult_EKF.Obv_GPS_update(poseEst2D.x(),poseEst2D.y(),GLOBV_RO);
        Lu_Matrix state = DR.glResult_EKF.getState();

        //输出定位的原始匹配结果，还是输出滤波定位结果
        if(OUTPUT_FILTERED_POSE) poseEst2D = CPose2D(state(0),state(1),state(2));
    }
    //若为保存点云模式，则用GPS的ekf结果作为位姿
    else if(SAVE_POINTCLOUD) poseEst2D = poseEkf2D;
    //若为生成占据栅格图模式，则用GPS的ekf结果作为位姿
    else if(USE_GPSFORMAPPING) poseEst2D = poseEkf2D;
    CPoint2D pointTemp(poseEst2D.x(),poseEst2D.y());
    poseEstDist2poseEKF = pointTemp.distance2DTo(poseEkf2D.x(),poseEkf2D.y());

    memUsageMb = (float)getMemoryUsage()/(1024*1024);  //what do you want to 

    if(SAVE_RESULTANDLOG)
    {
        outputFile_result.printf("%.2f\t%.2f\t%.2f\t%.5f\t%.2f\t%.2f\t%.5f\t%.3f\t%.2f\n",
            DR.mileage_sum,
            poseEkf2D.x()+output_pose_shift_x,
            poseEkf2D.y()+output_pose_shift_y,
            poseEkf2D.phi(),
            poseEst2D.x()+output_pose_shift_x,
            poseEst2D.y()+output_pose_shift_y,
            poseEst2D.phi(), 
            mainLoop.Tac()*1000,
            memUsageMb);
    }

    //发布定位结果数据
    //publish localization result in topic "pose_vlp16"
    { 
        std_msgs::Float64MultiArray pose_vlp16;
        double outLat, outLon;
        DR.xy2latlon(poseEst2D.x(),poseEst2D.y(), outLat, outLon);//xy2latlon
        pose_vlp16.data.push_back(poseEst2D.x()+output_pose_shift_x);     //x
        pose_vlp16.data.push_back(poseEst2D.y()+output_pose_shift_y);     //y
        pose_vlp16.data.push_back(poseEst2D.phi());   //phi
        pose_vlp16.data.push_back(outLat);            //lat
        pose_vlp16.data.push_back(outLon);            //lon
        pose_vlp16.data.push_back(poseEst2D.phi());   //phi
        pose_pub.publish(pose_vlp16);
    }

    //--------------------------------3D Window Display-------------------------------------------------------
    if(SHOW_WINDOW3D)
    {
        scene = COpenGLScene::Create();
        scene->getViewport()->setCustomBackgroundColor(TColorf(0.1,0.1,0.1));
        scene->insert(gridPlane);
        
        if(SHOW_GRIDMAPLOCAL)
        {
            objGridMap = CSetOfObjects::Create();
            localGridMap.getAs3DObject(objGridMap);
//            objGridMap->setColorA(0.9);
            objGridMap->setLocation(CPoint3D(0,0,-0.01));
            scene->insert(objGridMap);
        }

        if(SHOW_GLOBALPOINTCLOUD){
            global_point_cloud = CPointCloud::Create();
//            global_point_cloud->setColor(0.5,0.5,0.5);
            global_point_cloud->enableColorFromZ(1);
            global_point_cloud->loadFromPointsMap(&globalPointsMap);
            scene->insert(global_point_cloud);
        }

        if(SHOW_POINTCLOUDCOLORED)
        {
//            if(!USE_GLOBALPOINTCLOUDFORMATCHING)
            {
                vector<float> point_x, point_y, point_z;
                mrpt::opengl::CPointCloudColouredPtr pointCloudDisplay =
                        mrpt::opengl::CPointCloudColoured::Create();
                pointCloudDisplay->loadFromPointsMap(&curPointsMapColored);
                curPointsMapColored.getAllPoints(point_x, point_y, point_z);
                int colorCode = 0;
                float darkness = 0.15;
                float limitHigh = pointsMap_heightMax;
                float limitLow = pointsMap_heightMin;
                float limitRange = limitHigh - limitLow;
                for (int i = 0; i<point_z.size(); i++)
                {
                    if (point_z[i]>limitHigh) colorCode = 0;
                    else if (point_z[i] < limitLow) colorCode = 1280;
                    else colorCode = (1 - (point_z[i] - limitLow) / limitRange) * 1280;
                    int rangeNum = colorCode / 256;
                    float colorNum = (colorCode % 256) / 256.0;
                    switch (rangeNum)
                    {
                        default:
                        case 0:	pointCloudDisplay->setPointColor_fast(i, 1, colorNum*(1-darkness)+darkness, 0+darkness); break;//G(0->255)
                        case 1:	pointCloudDisplay->setPointColor_fast(i, (1 - colorNum)*(1-darkness)+darkness, 1, 0+darkness); break;//R(255->0)
                        case 2:	pointCloudDisplay->setPointColor_fast(i, 0+darkness, 1, colorNum*(1-darkness)+darkness); break;//B(0->255)
                        case 3:	pointCloudDisplay->setPointColor_fast(i, 0+darkness, (1 - colorNum)*(1-darkness)+darkness, 1); break;//G(255->0)
                        case 4:	pointCloudDisplay->setPointColor_fast(i, colorNum*(1-darkness)+darkness, 0+darkness, 1); break;//R(0->255)
                        case 5: pointCloudDisplay->setPointColor_fast(i, 0+darkness, 0+darkness, 1); break;
                    }
                }
                pointCloudDisplay->setPointSize(1.5);
                pointCloudDisplay->setPose(poseEst2D);
                scene->insert(pointCloudDisplay);
            }
        }

        if(SHOW_ROBOTPOSE)
        {
            objAxis->setPose(poseEst2D);
            objAxis->setScale(2);
            scene->insert(objAxis);
        }

        if(SHOW_GROUNDTRUTHPATH)
        {
            CSetOfLinesPtr objPath = CSetOfLines::Create();
            objPath->appendLine(poseEkf2D_last.x(),poseEkf2D_last.y(),0,
                                poseEkf2D.x(),poseEkf2D.y(),0);
            poseEkf2D_last = poseEkf2D;
            objPath->setLineWidth(2);
            objPath->setColor(0.5,0.9,0.5);
            objSetPath->insert(objPath);
            scene->insert(objSetPath);
        }

        if(SHOW_ROBOTPATH)
        {
            if(!(poseEst2D_last.x()==0 && poseEst2D_last.y()==0) && initialGuessStableCounter>1)
            {
                if (isPoseEstGood)
                {
                    if (!(abs(poseEst2D_last.x() - poseEst2D.x())>3 || abs(poseEst2D_last.y() - poseEst2D.y())>3))
                    {
                        CSetOfLinesPtr objPath = CSetOfLines::Create();
                        objPath->appendLine(poseEst2D_last.x(),poseEst2D_last.y(),0,
                                            poseEst2D.x(),poseEst2D.y(),0);
                        objPath->setLineWidth(2);
                        if(poseEstDist2poseEKF<(poseErrorMax/2))
                            errorColor = TColorf((poseEstDist2poseEKF*2/poseErrorMax)*0.8,0.8,0);
                        else errorColor = TColorf(0.8,
                                                  (1-(2*poseEstDist2poseEKF/poseErrorMax))*0.8,//make it darker
                                                  0);//自身对rgb值的范围有判断
                        objPath->setColor(errorColor);
                        if(DR.velocity>0.001)objSetPath->insert(objPath);//有速度时才添加
                    }
                }
                else {
                    if (!(abs(poseEst2D_last.x() - poseEst2D.x())>3 || abs(poseEst2D_last.y() - poseEst2D.y())>3))
                    {
                        CSetOfLinesPtr objPath = CSetOfLines::Create();
                        objPath->appendLine(poseEst2D_last.x(),poseEst2D_last.y(),0,
                                            poseEst2D.x(),poseEst2D.y(),0);
                        objPath->setLineWidth(2);
                        objPath->setColor(0.2,0.2,1);
                        objSetPath->insert(objPath);
                        if(DR.velocity>0.001)objSetPath->insert(objPath);//有速度时才添加
                    }
                }
            }
            scene->insert(objSetPath);        
        }

        if(SHOW_GPSEKFPOSE)
        {
            objGpsEKF->setPose(poseEkf2D);
            objGpsEKF->setScale(2);
            scene->insert(objGpsEKF);
        }

        if(SHOW_MINIWINDOW)
        {
            COpenGLViewportPtr view_mini = scene->createViewport("view_mini");
            view_mini->setBorderSize(1);
            view_mini->setViewportPosition(0.75, 0.75, 0.24, 0.24);
            view_mini->setTransparent(false);//是否透明
            view_mini->setCustomBackgroundColor(TColorf(0.1,0.1,0.1));//小窗口背景颜色
            //小地图视角
            mrpt::opengl::CCamera &camMini = view_mini->getCamera();
            camMini.setZoomDistance(1.5);
            camMini.setAzimuthDegrees(-90);
            camMini.setElevationDegrees(90);
            camMini.setPointingAt(poseEkf2D.x(),poseEkf2D.y(),0);
            //camMini.setPointingAt(poseEst2D.x(),poseEst2D.y(),0);

            mrpt::opengl::CSetOfLinesPtr objPath = CSetOfLines::Create();
            objPath->appendLine(poseEst2D.x(),poseEst2D.y(),0,
                                poseEkf2D.x(),poseEkf2D.y(),0);
            objPath->setLineWidth(2);
            objPath->setColor(1,1,0);

            char error[50];
            sprintf(error,
                    "%.2f m\n%.2f degree",
                    poseEstDist2poseEKF,
                    abs(poseEkf2D.phi()-poseEst2D.phi())/PI*180);
            const string msg_error = error;

            opengl::CText3DPtr obj_text_3d = opengl::CText3D::Create();
            obj_text_3d->setString(msg_error);
            obj_text_3d->setScale(0.08);
            obj_text_3d->setColor(1,1,0);
            obj_text_3d->setPose(CPose2D(poseEkf2D.x()-0.6,poseEkf2D.y()-0.2,0));
            view_mini->insert( obj_text_3d );

//            view_mini->insert(grid_plane_xy);
            view_mini->insert(objPath);
            view_mini->insert(objAxis);
            view_mini->insert(objGpsEKF);
        }

        if(CAMERA_FOLLOW_ROBOT)
        {
            win3D.setCameraPointingToPoint((float)poseEst2D.x(),(float)poseEst2D.y(),0);
        }

        if(SHOW_GRIDBOXS)
        {
            scene->insert(objBoxes);              
        }

        double outLat, outLon;
        DR.xy2latlon(poseEst2D.x(),poseEst2D.y(), outLat, outLon);

        //状态打印到win3D
        char win3DMsgState[200];
        sprintf(win3DMsgState,
                "X Y Heading: %.2f %.2f %.2f\nLat Lon: %.6f %.6f\nSpeed: %.2f m/s\nMainProc: %.1f ms",
                poseEst2D.x()+output_pose_shift_x,
                poseEst2D.y()+output_pose_shift_x,
                poseEst2D.phi(),
                outLat,
                outLon,
                DR.velocity,
                mainLoop.Tac()*1000);
        const string textMsgState = win3DMsgState;
        win3D.addTextMessage(0.05,0.95,textMsgState,TColorf(1,1,0),0,MRPT_GLUT_BITMAP_HELVETICA_12);

        if(!(step % 50))
            ROS_INFO("memory usage: %.1f MB",memUsageMb);

        opengl::COpenGLScenePtr &ptrScene = win3D.get3DSceneAndLock();
        ptrScene = scene;
        win3D.unlockAccess3DScene();
        win3D.forceRepaint();
    }
//    tempPointsMap.clear();

}//end of velodyne_points callback

//根据车辆位置，判断当前载入的refereneceMap中心是否需要改变，如需要则返回true，并
bool pointcloud_receive::isGlobalGridMapCenterChange(double robot_x, double robot_y, double Center_x, double Center_y)
{
    int x, y;
    if (robot_x > 0) {
        x = int(robot_x + 100); //强制类型转换
        x = floor(x / 20) * 20;
    }
    else {
        x = robot_x - 10;
        x = ceil(x / 20) * 20;
    }
    if (robot_y > 0){
        y = robot_y + 10;
        y = floor(y / 20) * 20;
    }
    else {
        y = robot_y - 10;
        y = ceil(y / 20) * 20;
    }

    if (x != Center_x || y != Center_y){
        //gridMapCenterUpdate = true;
        curGridMapCenter = CPoint2D(x, y);
        return true;
    }
    else {
        //gridMapCenterUpdate = false;
        return false;
    }
}


bool  pointcloud_receive::myLoadFromBitmapFile(
    const std::string   &file,
    float           resolution,
    float           xCentralPixel,
    float           yCentralPixel)
{
    CImage      imgFl;
    if (!imgFl.loadFromFile(file,0))
        return false;

    return myLoadFromBitmap(imgFl,resolution, xCentralPixel, yCentralPixel);
}


bool  pointcloud_receive::myLoadFromBitmap(const mrpt::utils::CImage &imgFl, float resolution, float xCentralPixel, float yCentralPixel)
{
    size_t bmpWidth = imgFl.getWidth();
    size_t bmpHeight = imgFl.getHeight();
        // Resize grid:
    float new_x_max = (imgFl.getWidth() - xCentralPixel) * resolution;
    float new_x_min = - xCentralPixel * resolution;
    float new_y_max = (imgFl.getHeight() - yCentralPixel) * resolution;
    float new_y_min = - yCentralPixel * resolution;

    if((new_x_max>new_x_min) && (new_y_max>new_y_min)){

        localGridMap.setSize(new_x_min,new_x_max,new_y_min,new_y_max,resolution);

        // And load cells content:
        for (size_t x=0;x<bmpWidth;x++)
            for (size_t y=0;y<bmpHeight;y++)
            {
                float f = imgFl.getAsFloat(x,bmpHeight-1-y);
                f = std::max(0.01f,f);
                f = std::min(0.99f,f);
                localGridMap.setCell((int)x,(int)y,f);
            }

        return true;
    }
    else return false;
}

bool pointcloud_receive::isNan(float fN)
{
    return !(fN==fN);
}


//加载数据
void pointcloud_receive::grid_localization_init()
{
    CConfigFile configFile;
    if(excute_mode == 0) //累积点云并存储
    {
        configFile.setFileName("/home/guolindong/catkin_ws/src/grid_localization/config/config_Save_Pointcloud.ini");
        ROS_INFO("***EXCUTE_MODE = SAVE_POINT_CLOUD***");
    }    
    else if (excute_mode == 1)  //生成并存储占据栅格图
    {
        configFile.setFileName("/home/guolindong/catkin_ws/src/grid_localization/config/config_Generate_GridMap.ini");
        ROS_INFO("***EXCUTE_MODE = GENERATE_AND_SAVE_GRIDMAP***");
    }    
    else if (excute_mode == 2)
    {
        ROS_INFO("***EXCUTE_MODE = LOCALIZATION***");
        configFile.setFileName("/home/guolindong/catkin_ws/src/grid_localization/config/config_Localization.ini");
    }  //全局定位模式
        

    SHOW_WINDOW3D           = configFile.read_bool("BasicSettings","SHOW_WINDOW3D",1,false);
    SHOW_MINIWINDOW         = configFile.read_bool("BasicSettings","SHOW_MINIWINDOW",0,false);
    SHOW_CURRENTLASERPOINTS = configFile.read_bool("BasicSettings","SHOW_CURRENTLASERPOINTS",0,false);
    SHOW_POINTCLOUD         = configFile.read_bool("BasicSettings","SHOW_POINTCLOUD",0,false);
    SHOW_POINTCLOUDCOLORED  = configFile.read_bool("BasicSettings","SHOW_POINTCLOUDCOLORED",1,false);
    SHOW_LASERPOINTS        = configFile.read_bool("BasicSettings","SHOW_LASERPOINTS",0,false);
    SHOW_GRIDMAPLOCAL       = configFile.read_bool("BasicSettings","SHOW_GRIDMAPLOCAL",0,false);
    SHOW_GRIDBOXS           = configFile.read_bool("BasicSettings","SHOW_GRIDBOXS",0,false);
    SHOW_ROBOTPOSE          = configFile.read_bool("BasicSettings","SHOW_ROBOTPOSE",1,false);
    SHOW_ROBOTPATH          = configFile.read_bool("BasicSettings","SHOW_ROBOTPATH",0,false);
    SHOW_GROUNDTRUTHPATH    = configFile.read_bool("BasicSettings","SHOW_GROUNDTRUTHPATH",0,false);
    SHOW_GPSEKFPOSE         = configFile.read_bool("BasicSettings","SHOW_GPSEKFPOSE",0,false);
    SHOW_GLOBALPOINTCLOUD   = configFile.read_bool("BasicSettings","SHOW_GLOBALPOINTCLOUD",0,false);

    USE_GLOBALPOINTCLOUDFORMATCHING = configFile.read_bool("BasicSettings","USE_GLOBALPOINTCLOUDFORMATCHING",1,false);
    SAVE_RESULTANDLOG       = configFile.read_bool("BasicSettings","SAVE_RESULTANDLOG",1,false); ROS_INFO("SAVE_RESULTANDLOG = %d",SAVE_RESULTANDLOG);
    SAVE_POINTCLOUD         = configFile.read_bool("BasicSettings","SAVE_POINTCLOUD",0,false); ROS_INFO("SAVE_POINTCLOUD = %d",SAVE_POINTCLOUD);
    SAVE_POINTCLOUDSTEP     = configFile.read_int("BasicSettings", "SAVE_POINTCLOUDSTEP",1,false);
    LOAD_POINTCLOUD         = configFile.read_bool("BasicSettings","LOAD_POINTCLOUD",0,false); ROS_INFO("LOAD_POINTCLOUD = %d",LOAD_POINTCLOUD);
    //NAME_MAPFILE_PLY        = configFile.read_string("BasicSettings","NAME_MAPFILE_PLY","/home/hubing/ros-project/hubing_ws_new/src/grid_localization/point_cloud_file/global_zizhu.ply ",false);
    USE_GPSFORMAPPING       = configFile.read_bool("BasicSettings","USE_GPSFORMAPPING",0,false); ROS_INFO("USE_GPSFORMAPPING = %d",USE_GPSFORMAPPING);
    CAMERA_FOLLOW_ROBOT     = configFile.read_bool("BasicSettings","CAMERA_FOLLOW_ROBOT",1,false);
    GENERATE_GRIDMAP        = configFile.read_bool("BasicSettings","GENERATE_GRIDMAP",0,false);
    GENERATE_GRIDMAPFILE    = configFile.read_bool("BasicSettings","GENERATE_GRIDMAPFILE",0,false);

    OUTPUT_FILTERED_POSE    = configFile.read_bool("BasicSettings","OUTPUT_FILTERED_POSE",1,false);

    output_pose_shift_x     = configFile.read_float("BasicSettings","output_pose_shift_x",0,false);
    output_pose_shift_y     = configFile.read_float("BasicSettings","output_pose_shift_y",0,false);

    whichGpsForInitial      = configFile.read_int("BasicSettings","whichGpsForInitial",1,false);
    timeForGps              = configFile.read_float("BasicSettings","timeForGps",20,false);
    gpsInitialStableCounter = configFile.read_float("BasicSettings","gpsInitialStableCounter",50,false);

    minDisBetweenLaserPoints = configFile.read_float("BasicSettings","minDisBetweenLaserPoints",0.05,false);
    pointsMap_heightMin     = configFile.read_float("BasicSettings","pointsMap_heightMin",0,false);
    pointsMap_heightMax     = configFile.read_float("BasicSettings","pointsMap_heightMax",1.5,false);
    pointsMap_clipOutOfRange= configFile.read_float("BasicSettings","pointsMap_clipOutOfRange",40,false);
    pointsMap_clipCenterDistance = configFile.read_float("BasicSettings","pointsMap_clipCenterDistance",10,false);

    gridMap_enhanceStep     = configFile.read_float("BasicSettings","gridMap_enhanceStep",0.01,false);
    gridMap_pzFactor        = configFile.read_float("BasicSettings","gridMap_pzFactor",0.1,false);
    gridMap_resolution      = configFile.read_float("BasicSettings","gridMap_resolution",0.1,false);
    gridMap_cellPointsThreshold = configFile.read_float("BasicSettings","gridMap_cellPointsThreshold",10,false);
    gridMap_halfSize        = configFile.read_float("BasicSettings","gridMap_halfSize",40,false);

    pose_front_VLP16_x       = configFile.read_float("BasicSettings","pose_front_VLP16_x",0,false);
    pose_front_VLP16_y       = configFile.read_float("BasicSettings","pose_front_VLP16_y",0.56,false);
    pose_front_VLP16_z       = configFile.read_float("BasicSettings","pose_front_VLP16_z",2.045,false);
    pose_front_VLP16_yaw     = configFile.read_float("BasicSettings","pose_front_VLP16_yaw",0,false);
    pose_front_VLP16_pitch   = configFile.read_float("BasicSettings","pose_front_VLP16_pitch",0.02,false);
    pose_front_VLP16_roll    = configFile.read_float("BasicSettings","pose_front_VLP16_roll",0.01,false);

    pose_rear_VLP16_x       = configFile.read_float("BasicSettings","pose_rear_VLP16_x",0,false);
    pose_rear_VLP16_y       = configFile.read_float("BasicSettings","pose_rear_VLP16_y",0.56,false);
    pose_rear_VLP16_z       = configFile.read_float("BasicSettings","pose_rear_VLP16_z",2.045,false);
    pose_rear_VLP16_yaw     = configFile.read_float("BasicSettings","pose_rear_VLP16_yaw",0,false);
    pose_rear_VLP16_pitch   = configFile.read_float("BasicSettings","pose_rear_VLP16_pitch",0.02,false);
    pose_rear_VLP16_roll    = configFile.read_float("BasicSettings","pose_rear_VLP16_roll",0.01,false);

    poseErrorMax    = configFile.read_float("BasicSettings","poseErrorMax", 5, false);

    icp_maxIterationFirst = configFile.read_int("BasicSettings", "icp_maxIterationFirst", 50,false);
    icp_maxIterationAfterFirst = configFile.read_int("BasicSettings", "icp_maxIterationAfterFirst", 2,false);

    icp.options.loadFromConfigFile(configFile, "ICP");

    //若用GPS生成点云
    if(SAVE_POINTCLOUD) LOAD_POINTCLOUD = false;
    //在生成点云的基础上，载入点云，并用gps结果生成占据栅格
    if(USE_GPSFORMAPPING) {
        SAVE_POINTCLOUD = false;
        LOAD_POINTCLOUD = true;
    }

}
