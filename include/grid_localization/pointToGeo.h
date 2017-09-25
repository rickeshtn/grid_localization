#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
using namespace std;
const double EARTH_RAD_EQ = 6378.137; // in km

//  how to calc scale:
//	latmean=(p1.latlon[0]+p2.latlon[0]+p3.latlon[0])/3;
//  scale = cos(latmean*(M_PI/180.0));
class pointToGeo
{
public:
	double GPS_OriginX;// = 529246.000;
	double GPS_OriginY;// = 3496650.000;
	double GPS_OffsetX;// = -1.1;
	double GPS_OffsetY;// = 1.03;
	double Ellipse_L0;// = 120.4620;

public:
	double latlon[2];
	double coordinate[2];
	pointToGeo(){ latlon[0]=0.0; latlon[1]=0.0; coordinate[0]=0.0; coordinate[1]=0.0; 
		GPS_OriginX = 529246.000;
		GPS_OriginY = 3496650.000;
		GPS_OffsetX = -1.1;
		GPS_OffsetY = 1.03;
		Ellipse_L0 = 120.4620;
		}
    pointToGeo(double lat, double lon=0.0, double heng=0.0, double shu=0.0){ latlon[0]=lat; latlon[1]=lon; coordinate[0]=heng; coordinate[1]=shu;}
	pointToGeo(const pointToGeo& P){latlon[0]=P.latlon[0]; latlon[1]=P.latlon[1]; coordinate[0]=P.coordinate[0];coordinate[1]=P.coordinate[1];}
	~pointToGeo(){}
	void mercatorProj(double scale, pointToGeo origin=0.0);
    void mercatordeProj(double scale,  pointToGeo origin=0.0);
	void gps2meter(
		double & gpsInMeter_east, 
		double & gpsInMeter_north, 
		double Lat, 
		double Lon, 
		double Ellipse_L0, 
		double OriginX, 
		double OriginY, 
		double OffsetX, 
		double OffsetY
		);

};
ostream & operator<<(ostream & Out,pointToGeo &p);
istream & operator>>(istream & In,pointToGeo &p);