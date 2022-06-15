#include "pv.h"
#include "math.h"

void PVModelInit(PVpanel *panel, float Voc, float Isc, float Vmp, float Imp, float G, float T)
{
	//Initialization of known values
	panel->Voc = Voc;
	panel->Isc = Isc;
	panel->Vmp = Vmp;
	panel->Imp = Imp;
	panel->G = G;
	panel->T = T;

	//Initialize panel values for convergence
	//According to https://oa.upm.es/30693/1/2014ICREARA.pdf

	panel->Vthermal = k*(panel->T + 274.15) / q;

	panel->Rsh = 100;
	panel->Rs = 1;
	panel->I0 = 0;
	panel->a = 1;
	panel->Ns = 1;
	panel->Np = 1;
	panel->idealityFactor = 1;
	panel->Ipv = 0;

    //Calculate a including Number of series and parallel cells
	panel->Ns = panel->Voc / UNITY_CELL_VOC;
	if (panel->Ns - floor(panel->Ns) > 0.5) panel->Ns = floor(panel->Ns) + 1;
	else panel->Ns = floor(panel->Ns);

	panel->Np = panel->Isc / UNITY_CELL_ISC;
	if (panel->Np - floor(panel->Np) > 0.5) panel->Np = floor(panel->Np) + 1;
	else panel->Np = floor(panel->Np);

	panel->a = panel->Ns / panel->Np;
	panel->a *= panel->idealityFactor;


	//Calculate Rs based on the above mentioned paper
	for (int i = 0; i < MAX_ITERATIONS; i++)
	{
		double eq_10_num = panel->a * panel->Vthermal * panel->Vmp * (2*panel->Imp - panel->Isc);
		double eq_10_den = (panel->Vmp*panel->Isc + panel->Voc*(panel->Imp - panel->Isc)) * (panel->Vmp - panel->Imp*panel->Rs) - panel->a*panel->Vthermal*(panel->Vmp*panel->Isc - panel->Voc*panel->Imp);
		panel->Rs = (panel->a * panel->Vthermal * log(eq_10_num / eq_10_den) + panel->Voc - panel->Vmp) / panel->Imp;
	}

	//Calculate Rsh based on the above mentioned paper
	float eq_11_num = (panel->Vmp * panel->Imp*panel->Rs) * (panel->Vmp - panel->Rs*(panel->Isc - panel->Imp) - panel->a*panel->Vthermal);
	float eq_11_den = (panel->Vmp - panel->Imp*panel->Rs) * (panel->Isc - panel->Imp) - panel->a*panel->Vthermal*panel->Imp;

	panel->Rsh = eq_11_num/eq_11_den;

	//calculate I0
	float eq_6_num = (panel->Rsh + panel->Rs)*panel->Isc - panel->Voc;
	float eq_6_den = panel->Rsh * expf(panel->Voc / (panel->a*panel->Vthermal));

	panel->I0 = eq_6_num / eq_6_den;

	//Calculate photocurrent Ipv
	//To Ipv exei eksarthsh kai apo to G kai apo thn gwnia prosptvshs kai apo ta arxidia moy
	//Tha to kanw pio meta
	panel->Ipv = ((panel->Rsh + panel->Rs) / panel->Rsh)*panel->Isc;

	//Fill V array and zero the I array
	for (int i = 0; i < NUMBER_OF_POINTS; i++)
	{
		panel->voltageLookUpTable[i] = i*Voc/(NUMBER_OF_POINTS - 1);
		panel->currentLookUpTable[i] = 0;
	}

	for (int i = 0; i < NUMBER_OF_POINTS; i++)
	{
		for (int j = 0; j < MAX_ITERATIONS; j++)
		{
			float exponent_value = (panel->voltageLookUpTable[i] + panel->currentLookUpTable[i]*panel->Rs) / (panel->a*panel->Vthermal);
			float term1 = panel->I0*(expf(exponent_value) - 1);
			float term2 = (panel->voltageLookUpTable[i] + panel->currentLookUpTable[i]*panel->Rs) / panel->Rsh;
			panel->currentLookUpTable[i] = panel->Ipv - term1 - term2;
		}
	}

}

float PVModelGetCurrentFromVoltage(PVpanel *panel, float voltage)
{
	return -1;
}

void PVModelUpdateModelRealTime(PVpanel *old_panel, PVpanel *new_panel)
{

}
