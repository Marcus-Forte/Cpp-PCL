/*
 * utm.h
 *
 *  Created on: 12 de nov de 2019
 *      Author: projeto
 */

#ifndef INCLUDE_GPAR_DJI_UTM_H_
#define INCLUDE_GPAR_DJI_UTM_H_

#include <cmath>


struct NavSatFix {
    double latitude;
    double longitude;
    double altitude;
};

struct Vector3 {
    double x;
    double y;
    double z;
};


#define C_PI (double)3.141592653589793


#define UTM_ZONE 24.0f // PECEM , UFC estão na zona 24M


//Centro da pilha 2, só pra ter ideia
#define UTM_X0 513384.87
#define UTM_Y0 9603813.42

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;


// Formula do site -> https://www.movable-type.co.uk/scripts/latlong-utm-mgrs.html
// Precisão levemente diferente da do site, mais ainda excepcionalmente precisa

// Este método gera coordenadas ** ENU **  (x _> East, y-> North, z -> Up)
Vector3 LatLong2Utm(const NavSatFix Gps){
Vector3 Position;

static const double falseEasting = 500e3;
static const double falseNorthing = 10000e3;
static const double k0 = 0.9996;
static const double f = 1/298.257223563;
static const double a = 6378137;
static const double e = sqrt(f*(2-f));
static const double n = f / (2-f);
static const double n2 = n*n,n3=n2*n,n4=n3*n,n5=n4*n,n6=n5*n;
static const double A = a/(1+n) * (1 + 1/4*n2 + 1/64*n4 + 1/256*n6);
static const double Long0 = ((UTM_ZONE-1)*6 - 180 + 3)*deg2rad;

double latitude = deg2rad*Gps.latitude;
double longitude = deg2rad*Gps.longitude - Long0;
double cos_longitude = cos(longitude);
double sin_longitude = sin(longitude);

static const double alfa[] = //ordem 6
{
		            1/2*n - 2/3*n2 + 5/16*n3 +   41/180*n4 -     127/288*n5 +      7891/37800*n6,
		                  13/48*n2 -  3/5*n3 + 557/1440*n4 +     281/630*n5 - 1983433/1935360*n6,
		                           61/240*n3 -  103/140*n4 + 15061/26880*n5 +   167603/181440*n6,
		                                   49561/161280*n4 -     179/168*n5 + 6601661/7257600*n6,
		                                                     34729/80640*n5 - 3418889/1995840*n6,
		                                                                  212378941/319334400*n6
};

double t = tan(latitude);
double sigma = sinh(e*atanh(e*t/sqrt(1+t*t)));
t = t*sqrt(1+sigma*sigma) - sigma*sqrt(1+t*t);
//double t = sinh(atanh(sin(latitude))-2*sqrt(n)/(1+n)*atanh(2*sqrt(n)/(1+n)*sin(latitude)));
double ksi_l = atan2(t,cos_longitude);
double eta_l = atanh(sin_longitude/sqrt(t*t + cos_longitude*cos_longitude));

double ksi = ksi_l;
double eta = eta_l;

for(int j=1;j<=6;j++){
	ksi += alfa[j-1]*sin(2*j*ksi_l)*cosh(2*j*eta_l);
	eta += alfa[j-1]*cos(2*j*ksi_l)*sinh(2*j*eta_l);
}

double x = falseEasting +  k0*A*eta; // coordenadas UTM
double y =  falseNorthing + k0*A*ksi; //coordenadas UTM

//translada em relação à origem (gerar x,y coerentes)
Position.x = x;
Position.y = y;

return Position;
}










#endif /* INCLUDE_GPAR_DJI_UTM_H_ */
