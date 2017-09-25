
#include "pointToGeo.h"

void pointToGeo::mercatorProj(double scale,pointToGeo origin)
{
	coordinate[0]=scale*latlon[1]*(M_PI/180.0)*EARTH_RAD_EQ-origin.coordinate[0];
	coordinate[1]=scale*EARTH_RAD_EQ*log(tan((90.0 + latlon[0]) * (M_PI/360.0)))-origin.coordinate[1];
}
void pointToGeo::mercatordeProj(double scale, pointToGeo origin)
{
	latlon[0]=360.0/M_PI*atan(exp((coordinate[1]+origin.coordinate[1])/(scale*EARTH_RAD_EQ)))-90.0;
	latlon[1]=180.0*(coordinate[0]+origin.coordinate[0])/(scale*EARTH_RAD_EQ*M_PI);
}
//ostream& operator<<(ostream& Out,pointToGeo &p){Out<<"the latlon of the point is ["<<p.latlon[0]<<","<<p.latlon[1]<<"], and the coordinate is ["<<p.coordinate[0]<<','<<p.coordinate[1]<<']'<<endl; return Out;}
//istream& operator>>(istream& In,pointToGeo &p){In>>p.latlon[0]>>p.latlon[1]; return In;}

void pointToGeo::gps2meter(double & gpsInMeter_east, double & gpsInMeter_north, double Lat, double Lon, double Ellipse_L0, double OriginX, double OriginY, double OffsetX, double OffsetY)
{
	static const double Ellipse_a = 6378137;
	static const double Ellipse_b = 6356752.3142;
	static const double PI = 3.14159265358;
	static const double Ellipse_n = (Ellipse_a - Ellipse_b) / (Ellipse_a + Ellipse_b);
	static const double Ellipse_e = sqrt(Ellipse_a*Ellipse_a - Ellipse_b*Ellipse_b) / Ellipse_a;
	static const double Ellipse_ee = sqrt(Ellipse_a*Ellipse_a - Ellipse_b*Ellipse_b) / Ellipse_b;
	static const double Ellipse_C0 = (Ellipse_a + Ellipse_b)*(1 + 0.25*pow(Ellipse_n, 2) + 0.015625*pow(Ellipse_n, 4))*0.5;
	static const double Ellipse_C1 = -1.5*Ellipse_n + 0.5625*pow(Ellipse_n, 3) - 0.09375*pow(Ellipse_n, 5);
	static const double Ellipse_C2 = 0.9375*pow(Ellipse_n, 2) - 0.46875*pow(Ellipse_n, 4);
	static const double Ellipse_C3 = -35 / 48 * pow(Ellipse_n, 3) + 0.41015625*pow(Ellipse_n, 5);
	static const double Ellipse_C4 = 0.615234375*pow(Ellipse_n, 4);

	double Ellipse_lat = Lat*PI / 180;
	double Ellipse_lon = (Lon - Ellipse_L0)*PI / 180;
	double Ellipse_N = Ellipse_a / sqrt(1 - pow(Ellipse_e*sin(Ellipse_lat), 2));
	double Ellipse_t = tan(Ellipse_lat);
	double Ellipse_g = Ellipse_ee*cos(Ellipse_lat);
	double Ellipse_m = cos(Ellipse_lat)*Ellipse_lon;

	double Ellipse_X = Ellipse_C0*(Ellipse_lat + Ellipse_C1*sin(2 * Ellipse_lat) + Ellipse_C2*sin(4 * Ellipse_lat) + Ellipse_C3*sin(6 * Ellipse_lat) + Ellipse_C4*sin(8 * Ellipse_lat));
	//=================================solution==================================

	double tempy = Ellipse_X + 0.5*Ellipse_N*Ellipse_t*pow(Ellipse_m, 2);
	tempy += 0.041666666666666666666666666667*Ellipse_N*Ellipse_t*(5 - pow(Ellipse_t, 2) + 9 * pow(Ellipse_g, 2) + 4 * pow(Ellipse_g, 4))*pow(Ellipse_m, 4);
	tempy += 0.0013888888888888888888888888889*Ellipse_N*Ellipse_t*(61 - 58 * pow(Ellipse_t, 2) + pow(Ellipse_t, 4) + 270 * pow(Ellipse_g, 2) - 330 * pow(Ellipse_g, 2)*pow(Ellipse_t, 2))*pow(Ellipse_m, 6);
	tempy += 0.0000248015873015873*Ellipse_t*Ellipse_N*pow(Ellipse_m, 8)*(1385 - 3111 * pow(Ellipse_t, 2) + 543 * pow(Ellipse_t, 4) - pow(Ellipse_t, 6));

	double tempx = Ellipse_N*Ellipse_m + Ellipse_N*pow(Ellipse_m, 3)*(1 - pow(Ellipse_t, 2) + pow(Ellipse_g, 2))*0.16666666666666666666666666666666666667;
	tempx += Ellipse_N*(5 - 18 * pow(Ellipse_t, 2) + pow(Ellipse_t, 4) + 14 * pow(Ellipse_g, 2) - 58 * pow(Ellipse_g, 2)*pow(Ellipse_t, 2))*pow(Ellipse_m, 5)*0.008333333333333333333333333333;
	tempx += Ellipse_N*(61 - 479 * pow(Ellipse_t, 2) + 179 * pow(Ellipse_t, 4) - pow(Ellipse_t, 6))*pow(Ellipse_m, 7)*0.000198412698412698 + 500000;

	tempx = tempx * 100 - OriginX * 100;     //cm
	tempy = tempy * 100 - OriginY * 100;     //cm

	tempx += OffsetX * 100;
	tempy += OffsetY * 100;

	gpsInMeter_east = (tempx) / 100.0;   //m,x
	gpsInMeter_north = (tempy) / 100.0;   //m,y
}

