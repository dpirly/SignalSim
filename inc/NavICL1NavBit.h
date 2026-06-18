//----------------------------------------------------------------------
// NavICL1NavBit.h:
//   Declaration of navigation bit synthesis class for NavIC L1-SPS NAV
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __NAVIC_L1_NAV_BIT_H__
#define __NAVIC_L1_NAV_BIT_H__

#include "NavICL1Ldpc.h"
#include "NavICLNavBit.h"

#include <stdint.h>

class NavICL1NavBit : public NavICLNavBit
{
public:
	NavICL1NavBit();
	~NavICL1NavBit();

	int GetFrameData(GNSS_TIME StartTime, int svid, int Param, int *NavBits);

private:
	void ComposeSf1(int Toi, int NavBits[]);
	void ComposeSf2(PGPS_EPHEMERIS Eph, int Itow, int Bits[]);
	void ComposeSf3(int FrameCount, int Bits[]);
	void ComposeSf3Mt5(int Bits[]);
	void ComposeSf3Mt6(int FrameCount, int Bits[]);
	void ComposeSf3Mt8(int Bits[]);
	void ComposeSf3Mt10(int Bits[]);
	void ComposeSf3Mt17(int Bits[]);
	void AppendCrc(int Bits[], int DataBits);
	void SetBits(int Bits[], int Pos, int Width, uint64_t Value);
	void SetSignedBits(int Bits[], int Pos, int Width, int64_t Value);
	int64_t ScaleSigned(double Value, int Power);
	uint64_t ScaleUnsigned(double Value, int Power);
	int64_t ScaleSemiCircleSigned(double Value, int Power);
	uint64_t ScaleSemiCircleUnsigned(double Value, int Power);
	int EncodeLdpc(const int DataBits[], int DataLength, int CodeBits[]);
	void BlockInterleave(const int Sf2Symbols[], const int Sf3Symbols[], int NavBits[]);
	void AddLdpcEdges(const NavICL1LdpcEntry Entries[], int Count, int RowOffset, int ColOffset,
		const int DataBits[], int DataLength, int MatrixBits[], int Rhs[]);
	int SolveLdpc(int Rows, int MatrixBits[], int Rhs[], int Solution[]);
};

#endif // __NAVIC_L1_NAV_BIT_H__
