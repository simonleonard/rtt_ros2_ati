#ifndef FTATI_H
#define FTATI_H

#include "ftconfig.h"

#define defaultfile "../FT12538.cal"
#define index 1

class FTATI 
{
public:
	FTATI(char * filepath);// = defaultfile);
	~FTATI();
	void setBias(float * zeroArray);
	void Convert(float * voltages,float * output);
private:
	Calibration *cal;
};



#endif
