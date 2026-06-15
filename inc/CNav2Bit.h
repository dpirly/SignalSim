//----------------------------------------------------------------------
// CNav2Bit.h:
//   Declaration of navigation bit synthesis class for CNAV2
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __CNAV2_BIT_H__
#define __CNAV2_BIT_H__

#include "NavBit.h"

#define L1C_SUBFRAME2_SYMBOL_LENGTH 600
#define L1C_SUBFRAME3_SYMBOL_LENGTH 274
#define CNAV2_EPHEMERIS_SLOTS 42

#define PAGE_INDEX_IONO_UTC 0	// Subframe3 index 0 for ionosphere and UTC parameters
#define PAGE_INDEX_REDUCED_ALM 1	// Subframe3 index start from 1 for reduced-almanac
#define PAGE_INDEX_MIDI_ALM 10	// Subframe3 index start from 10 for midi-almanac

class CNav2Bit : public NavBit
{
public:
	CNav2Bit();
	~CNav2Bit();

	int GetFrameData(GNSS_TIME StartTime, int svid, int Param, int *NavBits);
	int SetEphemeris(int svid, PGPS_EPHEMERIS Eph);
	int SetAlmanac(GPS_ALMANAC Alm[]) { return 0; };
	int SetIonoUtc(PIONO_PARAM IonoParam, PUTC_PARAM UtcParam);

private:
	unsigned int Subframe2[CNAV2_EPHEMERIS_SLOTS][18];	// GPS 1-32 plus QZSS 193-202, 576bits in subframe 2, 32bits in each DWORD MSB first, lowest address first
	unsigned int Subframe3[64][8];	// reserve 64 250bit subframe3 contents (8bit PRN filled with 0), 32bits in each DWORD MSB first, lowest address first
	unsigned int ISC[CNAV2_EPHEMERIS_SLOTS][2];	// ISC for GPS/QZSS SVs, ISC_L1C/A and ISC_L2C in 26LSB of [0], ISC_L5I5 and ISC_L5Q5 in 26LSB of [1]

	static const unsigned long long BCH_toi_table[256];	// BCH encode table for TOI
	static const unsigned int L1CMatrixGen2[L1C_SUBFRAME2_SYMBOL_LENGTH*19];
	static const unsigned int L1CMatrixGen3[L1C_SUBFRAME3_SYMBOL_LENGTH*9];

	void ComposeSubframe2(PGPS_EPHEMERIS Eph, unsigned int Subframe2[18], unsigned int ISCData[2]);
	int GetSvidIndex(int svid);
	void LDPCEncode(unsigned int Stream[], int bits[], int SymbolLength, int TableSize, const unsigned int MatrixGen[]);
	int XorBits(unsigned int Data);
	void GetSubframe3Data(int Svid, int PageIndex, unsigned int Subframe3Data[9]);
};

#endif // __CNAV2_BIT_H__
