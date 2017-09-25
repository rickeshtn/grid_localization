#ifndef DISPLAY_DATA_H
#define DISPLAY_DATA_H


#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/math/utils.h>

#include <mrpt/maps/COctoMap.h>
#include"mrpt/gui.h"

#include"mrpt/opengl.h"
#include"mrpt/opengl/CPlanarLaserScan.h"
#include"opencv2/opencv.hpp"

#include"mrpt/system/os.h"

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::bayes;
using namespace std;
using namespace cv;
#include"virtual_scan_type.h"
class display_data_type
{

//    CDisplayWindowPlots win2D;

    COpenGLScenePtr	scene;
    mrpt::opengl::CGridPlaneXYPtr groundPlane;
    mrpt::opengl::CArrowPtr virtual_scan_arrow;
    mrpt::opengl::CAssimpModelPtr suvModel;
    opengl::CSetOfObjectsPtr obj_gridMap;
    mrpt::opengl::CPlanarLaserScanPtr obj_scan;
    mrpt::opengl::CSetOfLinesPtr    admissable_space_display;
public:
    display_data_type();
    virtual_scan_type   virtual_scan;
    CDisplayWindow3D win3D;
    void insert_grid();
    void insert_gridmap_local();
    void insert_suvModel();
    void insert_scan_center_arrow();
    void insert_scan_result();
    void insert_admissable_space();
    void refresh_3Ddisplay();
    void refresh_2Ddisplay();
    void clear_scene();
};

#endif // DISPLAY_DATA_H
