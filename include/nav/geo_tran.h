// Codes for geomatics applications (geo-coordinate projection and transformation)
// By Yue Pan et al.

#ifndef GEOTRAN_H
#define GEOTRAN_H

//#include <ogr_spatialref.h>
//#include "cpl_conv.h"
#include "utility.hpp"

#include <vector>
#include <list>

using namespace std;

namespace lo
{

class GeoTransform
{
  public:
	// Get the 6DOF pose from OXTS data
	// Here, we use proj4 library to do the UTM (Universe Tranverse Mecator) Projection
	// proj4 library: https://github.com/OSGeo/PROJ (Refer the website for download methods and 'readme')
	// Instead of the KITTI's projection calculation method
	// According to experiment, there are up to 5 meter difference between the two projection method in a local map (1km^2)
	// So we choose to use the standard UTM projection
	Eigen::Matrix4d GetTransform(gnss_info_t &gnss_data)
	{

		const static double global_scale = 0.863828;
		const static double earth_radius = 6378137.0; // unit m

		//double lat = gnss_info.lat * M_PI / 180;
		//double lon = gnss_info.lon * M_PI / 180; //unit: rad
		double lat = gnss_data.lat; //unit:degree
		double lon = gnss_data.lon; //unit:degree
		double alt = gnss_data.alt;
		double roll = gnss_data.roll * M_PI / 180;
		double pitch = gnss_data.pitch * M_PI / 180;
		double yaw = gnss_data.yaw * M_PI / 180;

		// Calculate Rotation from roll, pitch, yaw
		Eigen::Matrix3d Rx(Eigen::Matrix3d::Identity());
		Rx(1, 1) = Rx(2, 2) = cos(roll);
		Rx(2, 1) = sin(roll);
		Rx(1, 2) = -Rx(2, 1);

		Eigen::Matrix3d Ry(Eigen::Matrix3d::Identity());
		Ry(0, 0) = Ry(2, 2) = cos(pitch);
		Ry(0, 2) = sin(pitch);
		Ry(2, 0) = -Ry(0, 2);

		Eigen::Matrix3d Rz(Eigen::Matrix3d::Identity());
		Rz(0, 0) = Rz(1, 1) = cos(yaw);
		Rz(1, 0) = sin(yaw);
		Rz(0, 1) = -Rz(1, 0);

		// A reference: Compare of the same single points' coordinate calculated by UTM and KITTI's projection
		// UTM 51 Zone Proj X:396595.067945 , Y:3414994.320534
		// KITTI Proj X:11723782.924684 , Y:3122787.514189

		// Use proj4 to do the UTM projection
		projPJ pj_merc, pj_latlong;

		// Notice the UTM projection zone
		// Shanghai/Hangzhou UTM-WGS84 Zone 51 N
		// Tokyo 54 N
		// Beijing/Shenzhen/HongKong 50 N

		// From WGS84 geodesy (latlon) system to WGS84 UTM map system (XYZ)
		if (!(pj_merc = pj_init_plus("+proj=utm +zone=51 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")))
			exit(1);
		if (!(pj_latlong = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs"))) //WGS84
			exit(1);
		double x_utm = lon * DEG_TO_RAD;
		double y_utm = lat * DEG_TO_RAD;

		int p = pj_transform(pj_latlong, pj_merc, 1, 1, &x_utm, &y_utm, NULL);

		//Free Memory
		pj_free(pj_merc);
		pj_free(pj_latlong);

		Eigen::Vector3d trans;
		trans[0] = x_utm;
		trans[1] = y_utm;
		trans[2] = alt;

		Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

		//TODO //Check the reason
		pose.block<3, 3>(0, 0) = Rz * Ry * Rx;
		// Should be z y x

		pose.block<3, 1>(0, 3) << trans[0], trans[1], trans[2];

		return pose;
	}

	//Convert [Longitude,Latitude,Elevation] Coordinate to [X,Y,Z] Coordinate by UTM projection under WGS84 System
	int BLH2XYZ_WGS84(const std::vector<double> &BLH_coord, std::vector<double> &XYZ_coord)
	{
		OGRSpatialReference *RefSource = new OGRSpatialReference;
		RefSource->SetWellKnownGeogCS("WGS84");

		OGRSpatialReference *RefTarget = new OGRSpatialReference;
		RefTarget = RefSource->CloneGeogCS();

		int utmzone = BLH_coord[1] / 6 + 31; //six degree zone

		RefTarget->SetProjCS("UTM(WGS84) in northern hemisphere.");
		RefTarget->SetUTM(utmzone, TRUE);
		OGRCoordinateTransformation *poTransform = OGRCreateCoordinateTransformation(RefSource, RefTarget);

		double tempX = BLH_coord[1];
		double tempY = BLH_coord[0];
		double tempZ = BLH_coord[2];

		poTransform->Transform(1, &tempX, &tempY, &tempZ);

		XYZ_coord.resize(3);

		XYZ_coord[0] = tempX;
		XYZ_coord[1] = tempY;
		XYZ_coord[2] = tempZ;

		return utmzone;
	}

	//Convert [X,Y,Z] Coordinate to [Longitude,Latitude,Elevation] Coordinate by UTM inverse projection under WGS84 System with given UTM zone number.
	void XYZ2BLH_WGS84(const std::vector<double> &XYZ_coord, const int utmzone, std::vector<double> &BLH_coord)
	{
		OGRSpatialReference *RefSource = new OGRSpatialReference;
		RefSource->SetWellKnownGeogCS("WGS84");
		RefSource->SetProjCS("UTM(WGS84) in northern hemisphere.");
		RefSource->SetUTM(utmzone, TRUE);

		OGRSpatialReference *RefTarget = new OGRSpatialReference;
		RefTarget = RefSource->CloneGeogCS();

		OGRCoordinateTransformation *poTranform = OGRCreateCoordinateTransformation(RefSource, RefTarget);

		double tempx = XYZ_coord[0];
		double tempy = XYZ_coord[1];
		double tempz = XYZ_coord[2];

		poTranform->Transform(1, &tempx, &tempy, NULL);
		BLH_coord.resize(3);
		BLH_coord[1] = tempx; //Longitude
		BLH_coord[0] = tempy; //Latitude
		BLH_coord[2] = tempz; //Elevation
	}

	void XYZ2BLH_ENG(const std::vector<double> &XYZ_coord, float centerlong, std::vector<double> &BLH_coord)
	{
		//CPLSetConfigOption("GDAL_DATA", "F:\\4_softwares\\gdaldata");
		OGRSpatialReference *RefSource = new OGRSpatialReference;
		RefSource->SetWellKnownGeogCS("EPSG:4490"); //CGCS2000

		//Other Common Used CS's code in EPSG
		//spatialReference.importFromEPSG(4326);//WGS84
		//spatialReference.importFromEPSG(4214);//BeiJing54
		//spatialReference.importFromEPSG(4610);//XIAN80
		//spatialReference.importFromEPSG(4490);//CGCS2000

		RefSource->SetProjCS("CGCS2000/UTM");
		RefSource->SetTM(0, centerlong, 1.0, 500000, 0); // centerlong 104 24' here,  104 + 24.0 / 60   //Universal

		OGRSpatialReference *RefTarget = new OGRSpatialReference;
		RefTarget = RefSource->CloneGeogCS();

		OGRCoordinateTransformation *poTranform = OGRCreateCoordinateTransformation(RefSource, RefTarget);

		double tempx = XYZ_coord[0];
		double tempy = XYZ_coord[1];
		double tempz = XYZ_coord[2];

		poTranform->Transform(1, &tempx, &tempy, NULL);
		BLH_coord.resize(3);
		BLH_coord[1] = tempx; //Longitude
		BLH_coord[0] = tempy; //Latitude
		BLH_coord[2] = tempz; //Elevation
	}

	void BLH2XYZ_CGCS(const std::vector<double> &BLH_coord, float centerlong, float proj_surface_h_eng, std::vector<double> &XYZ_coord)
	{
		//CGCS2000 Parameters
		const double a = 6378137.0;
		const double b = 6356752.314;
		//const double R = 6378245.0;
		const double shift = 500000.0;

		//CPLSetConfigOption("GDAL_DATA", "F:\\4_softwares\\gdaldata");
		OGRSpatialReference *RefSource = new OGRSpatialReference;
		RefSource->SetWellKnownGeogCS("EPSG:4490"); //CGCS2000

		//Other Common Used CS's code in EPSG
		//spatialReference.importFromEPSG(4326);//WGS84
		//spatialReference.importFromEPSG(4214);//BeiJing54
		//spatialReference.importFromEPSG(4610);//XIAN80
		//spatialReference.importFromEPSG(4490);//CGCS2000

		OGRSpatialReference *RefTarget = new OGRSpatialReference;
		RefTarget = RefSource->CloneGeogCS();
		RefTarget->SetProjCS("CGCS2000/UTM");
		RefTarget->SetTM(0, centerlong, 1.0, (int)shift, 0); // centerlong 104 24' here,  104 + 24.0 / 60   //Universal 0.9996

		OGRCoordinateTransformation *poTranform = OGRCreateCoordinateTransformation(RefSource, RefTarget);

		double tempX = BLH_coord[1];
		double tempY = BLH_coord[0];
		double tempZ = BLH_coord[2];
		double R = MeanRofEarth(a, b, tempY);
		cout << "R is " << R << endl;
		poTranform->Transform(1, &tempX, &tempY, &tempZ);

		XYZ_coord.resize(3);

		XYZ_coord[0] = (tempX - shift) * (1 + proj_surface_h_eng / R) + shift;
		XYZ_coord[1] = tempY * (1 + proj_surface_h_eng / R);
		XYZ_coord[2] = tempZ;
	}
	//Our requirement;
	//ENG XYZ ->> CGCS BLH ->> UTM XYZ;
	//XYZ2BLH_ENG(XYZ_eng_coord,centerlong,BLH_coord);
	//BLH2XYZ_WGS84(BLH_coord,XYZ_utm_coord);

	//Try 4DOF Coordinate Transformation.

  protected:
	double MeanRofEarth(double a, double b, double B)
	{
		const double pi_ = 3.141592654;
		B = B / 180 * pi_;
		double e0 = sqrt((a * a - b * b) / a / a);
		//double ep = e0*a/b;
		double W = sqrt(1 - e0 * e0 * sin(B) * sin(B));
		double N = a / W;
		double M = a * (1 - e0 * e0) / W / W / W;
		double R = sqrt(N * M);
		return R;
	}

  private:
};
} // namespace lo

#endif //GEOTRAN_H