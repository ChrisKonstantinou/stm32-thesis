#ifndef __PV_H
#define __PH_H

#include "main.h"

typedef struct
{
	uint16_t a;

} PVpanel;

void PVModelInit(PVpanel *);
void PVModelUpdateModelRealTime(PVpanel *, PVpanel *);
float PVModelGetCurrentFromVoltage(PVpanel *, float);

#endif
