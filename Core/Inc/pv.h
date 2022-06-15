#ifndef _PV_H
#define _PH_H

#define NUMBER_OF_POINTS 20000

#define k 1.38064852e-23
#define q 1.602176634e-19
#define MAX_ITERATIONS 50
#define RANGE_FACTOR 1.25

#define UNITY_CELL_VOC 0.7
#define UNITY_CELL_ISC 8.5


#include "main.h"


typedef struct
{
	//float voltageLookUpTable[NUMBER_OF_POINTS];
	float currentLookUpTable[NUMBER_OF_POINTS];

	float Voc;
	float Isc;
	float Vmp;
	float Imp;

	float G;
	float T;

	float Vthermal;
	float Rsh;
	float Rs;
	float I0;
	float a;
	float Ns;
	float Np;
	float idealityFactor;

	float Ipv;

} PVpanel;

void PVModelInit(PVpanel *, float, float, float, float, float, float);
void PVModelUpdateModelRealTime(PVpanel *, PVpanel *);
float PVModelGetCurrentFromVoltage(PVpanel *, float);

#endif
