#ifndef NIUSB_H
#define NIUSB_H

#include "NIDAQmxBase.h"

class NIUSB
{
public:
    NIUSB(uInt32 sampleSize = 1, float64 readRate = 60);
    ~NIUSB();
    void Start();
    int Read(int32 pointsnum, float64* data, uInt32 arraysize);
    void Stop();
	uInt32 getSampleSize();

private:
    char chan[];
    TaskHandle nihandle;
	uInt32 samples;
	float64 rate;
};

#endif // NIUSB_H
