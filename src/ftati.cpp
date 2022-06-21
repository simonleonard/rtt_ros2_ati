#include "ftati.h"
#include "ftconfig.h"
#include <iostream>
#include <stddef.h>

using namespace std;

FTATI::FTATI(char * filepath)
{	
	cal = createCalibration(filepath,index);
	SetForceUnits(cal,"N");
	SetTorqueUnits(cal,"N-m");
}
FTATI::~FTATI()
{
	if (cal != NULL)
	{
		destroyCalibration(cal);
	}
}

void FTATI::setBias(float * zeroArray)
{
/*	cout << "pre Bias call" << endl
	<< cal << endl
	<<zeroArray << endl;*/
	Bias(cal,zeroArray);
}

void FTATI::Convert(float * voltages, float * output)
{
	ConvertToFT(cal,voltages,output);
}

