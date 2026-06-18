//----------------------------------------------------------------------
// NavICLNavBit.h:
//   Declaration of navigation bit synthesis class for NavIC L5-SPS NAV
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __NAVIC_LNAV_BIT_H__
#define __NAVIC_LNAV_BIT_H__

#include "NavBit.h"

class NavICLNavBit : public NavBit
{
public:
	NavICLNavBit();
	~NavICLNavBit();

	int GetFrameData(GNSS_TIME StartTime, int svid, int Param, int *NavBits);
	int SetEphemeris(int svid, PGPS_EPHEMERIS Eph);
	int SetAlmanac(GPS_ALMANAC Alm[]);
	int SetIonoUtc(PIONO_PARAM IonoParam, PUTC_PARAM UtcParam);

protected:
	GPS_EPHEMERIS Ephemeris[14];
	unsigned char EphValid[14];
	GPS_ALMANAC Almanac[14];
	IONO_PARAM IonoParam;
	UTC_PARAM UtcParam;

	void ComposeSubframe(int Subframe, int TowCount, int svid, int Bits[]);
	void ComposeSubframe1(PGPS_EPHEMERIS Eph, int TowCount, int Bits[]);
	void ComposeSubframe2(PGPS_EPHEMERIS Eph, int TowCount, int Bits[]);
	void ComposeSubframe3(int TowCount, int Bits[]);
	void ComposeSubframe4(int TowCount, int Bits[]);
	void AppendCrc(int Bits[]);
	void EncodeFrame(const int Bits[], int NavBits[]);
	void SetBits(int Bits[], int Pos, int Width, unsigned int Value);
	void SetSignedBits(int Bits[], int Pos, int Width, int Value);
	int ScaleSigned(double Value, int Power);
	unsigned int ScaleUnsigned(double Value, int Power);
	int ScaleSemiCircleSigned(double Value, int Power);
	unsigned int ScaleSemiCircleUnsigned(double Value, int Power);
};

#endif // __NAVIC_LNAV_BIT_H__
