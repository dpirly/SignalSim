//----------------------------------------------------------------------
// NavICLNavBit.cpp:
//   Implementation of navigation bit synthesis class for NavIC L5-SPS NAV
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <math.h>
#include <memory.h>

#include "ConstVal.h"
#include "NavICLNavBit.h"

#define NAVIC_LNAV_DATA_BITS 286
#define NAVIC_LNAV_CRC_DATA_BITS 262
#define NAVIC_LNAV_CONV_BITS 292
#define NAVIC_LNAV_SYMBOL_BITS 584
#define NAVIC_LNAV_FRAME_BITS 600

NavICLNavBit::NavICLNavBit()
{
	memset(Ephemeris, 0, sizeof(Ephemeris));
	memset(EphValid, 0, sizeof(EphValid));
	memset(&IonoParam, 0, sizeof(IonoParam));
	memset(&UtcParam, 0, sizeof(UtcParam));
}

NavICLNavBit::~NavICLNavBit()
{
}

int NavICLNavBit::GetFrameData(GNSS_TIME StartTime, int svid, int Param, int *NavBits)
{
	int Bits[NAVIC_LNAV_DATA_BITS];
	int WeekMs;
	int FrameCount;
	int TowCount;
	int Subframe;

	if (!NavBits)
		return 1;

	memset(Bits, 0, sizeof(Bits));
	memset(NavBits, 0, sizeof(int) * NAVIC_LNAV_FRAME_BITS);
	if (svid < 1 || svid > 14)
		return 1;

	WeekMs = StartTime.MilliSeconds;
	if (WeekMs < 0)
		WeekMs = (WeekMs % 604800000) + 604800000;
	WeekMs %= 604800000;
	FrameCount = WeekMs / 12000;
	TowCount = (FrameCount + 1) % (604800 / 12);
	Subframe = (FrameCount % 4) + 1;

	ComposeSubframe(Subframe, TowCount, svid, Bits);
	AppendCrc(Bits);
	EncodeFrame(Bits, NavBits);

	return 0;
}

int NavICLNavBit::SetEphemeris(int svid, PGPS_EPHEMERIS Eph)
{
	if (svid < 1 || svid > 14 || !Eph || !Eph->valid)
		return 0;

	Ephemeris[svid - 1] = *Eph;
	EphValid[svid - 1] = 1;
	return 0;
}

int NavICLNavBit::SetAlmanac(GPS_ALMANAC Alm[])
{
	if (Alm)
		memcpy(Almanac, Alm, sizeof(Almanac));
	return 0;
}

int NavICLNavBit::SetIonoUtc(PIONO_PARAM IonoParamIn, PUTC_PARAM UtcParamIn)
{
	if (IonoParamIn)
		IonoParam = *IonoParamIn;
	if (UtcParamIn)
		UtcParam = *UtcParamIn;
	return 0;
}

void NavICLNavBit::ComposeSubframe(int Subframe, int TowCount, int svid, int Bits[])
{
	PGPS_EPHEMERIS Eph = EphValid[svid - 1] ? (Ephemeris + svid - 1) : (PGPS_EPHEMERIS)0;

	if (!Eph)
	{
		SetBits(Bits, 8, 17, TowCount & 0x1ffff);
		SetBits(Bits, 27, 2, (Subframe - 1) & 0x3);
		return;
	}

	switch (Subframe)
	{
	case 1:
		ComposeSubframe1(Eph, TowCount, Bits);
		break;
	case 2:
		ComposeSubframe2(Eph, TowCount, Bits);
		break;
	case 3:
		ComposeSubframe3(TowCount, Bits);
		break;
	default:
		ComposeSubframe4(TowCount, Bits);
		break;
	}
}

void NavICLNavBit::ComposeSubframe1(PGPS_EPHEMERIS Eph, int TowCount, int Bits[])
{
	int i = 8;

	SetBits(Bits, i, 17, TowCount & 0x1ffff); i += 17 + 2;
	SetBits(Bits, i, 2, 0); i += 2 + 1;
	SetBits(Bits, i, 10, Eph->week & 0x3ff); i += 10;
	SetSignedBits(Bits, i, 22, ScaleSigned(Eph->af0, 31)); i += 22;
	SetSignedBits(Bits, i, 16, ScaleSigned(Eph->af1, 43)); i += 16;
	SetSignedBits(Bits, i, 8, ScaleSigned(Eph->af2, 55)); i += 8;
	SetBits(Bits, i, 4, Eph->ura & 0xf); i += 4;
	SetBits(Bits, i, 16, (Eph->toc / 16) & 0xffff); i += 16;
	SetSignedBits(Bits, i, 8, ScaleSigned(Eph->tgd, 31)); i += 8;
	SetSignedBits(Bits, i, 22, ScaleSemiCircleSigned(Eph->delta_n, 41)); i += 22;
	SetBits(Bits, i, 8, Eph->iode); i += 8 + 10;
	SetBits(Bits, i, 2, Eph->health & 0x3); i += 2;
	SetSignedBits(Bits, i, 15, ScaleSigned(Eph->cuc, 28)); i += 15;
	SetSignedBits(Bits, i, 15, ScaleSigned(Eph->cus, 28)); i += 15;
	SetSignedBits(Bits, i, 15, ScaleSigned(Eph->cic, 28)); i += 15;
	SetSignedBits(Bits, i, 15, ScaleSigned(Eph->cis, 28)); i += 15;
	SetSignedBits(Bits, i, 15, ScaleSigned(Eph->crc, 4)); i += 15;
	SetSignedBits(Bits, i, 15, ScaleSigned(Eph->crs, 4)); i += 15;
	SetSignedBits(Bits, i, 14, ScaleSemiCircleSigned(Eph->idot, 43));
}

void NavICLNavBit::ComposeSubframe2(PGPS_EPHEMERIS Eph, int TowCount, int Bits[])
{
	int i = 8;

	SetBits(Bits, i, 17, TowCount & 0x1ffff); i += 17 + 2;
	SetBits(Bits, i, 2, 1); i += 2 + 1;
	SetSignedBits(Bits, i, 32, ScaleSemiCircleSigned(Eph->M0, 31)); i += 32;
	SetBits(Bits, i, 16, (Eph->toe / 16) & 0xffff); i += 16;
	SetBits(Bits, i, 32, ScaleUnsigned(Eph->ecc, 33)); i += 32;
	SetBits(Bits, i, 32, ScaleUnsigned(Eph->sqrtA, 19)); i += 32;
	SetSignedBits(Bits, i, 32, ScaleSemiCircleSigned(Eph->omega0, 31)); i += 32;
	SetSignedBits(Bits, i, 32, ScaleSemiCircleSigned(Eph->w, 31)); i += 32;
	SetSignedBits(Bits, i, 22, ScaleSemiCircleSigned(Eph->omega_dot, 41)); i += 22;
	SetSignedBits(Bits, i, 32, ScaleSemiCircleSigned(Eph->i0, 31));
}

void NavICLNavBit::ComposeSubframe3(int TowCount, int Bits[])
{
	int i;

	SetBits(Bits, 8, 17, TowCount & 0x1ffff);
	SetBits(Bits, 27, 2, 2);
	SetBits(Bits, 30, 6, 11);
	i = 174;
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.a0, 30)); i += 8;
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.a1, 27)); i += 8;
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.a2, 24)); i += 8;
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.a3, 24)); i += 8;
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.b0, -11)); i += 8;
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.b1, -14)); i += 8;
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.b2, -16)); i += 8;
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.b3, -16));
}

void NavICLNavBit::ComposeSubframe4(int TowCount, int Bits[])
{
	int i;

	SetBits(Bits, 8, 17, TowCount & 0x1ffff);
	SetBits(Bits, 27, 2, 3);
	SetBits(Bits, 30, 6, 9);
	i = 36;
	SetSignedBits(Bits, i, 16, ScaleSigned(UtcParam.A0, 35)); i += 16;
	SetSignedBits(Bits, i, 13, ScaleSigned(UtcParam.A1, 51)); i += 13;
	SetSignedBits(Bits, i, 7, ScaleSigned(UtcParam.A2, 68)); i += 7;
	SetSignedBits(Bits, i, 8, UtcParam.TLS); i += 8;
	SetBits(Bits, i, 16, UtcParam.tot & 0xffff); i += 16;
	SetBits(Bits, i, 10, UtcParam.WN & 0x3ff); i += 10;
	SetBits(Bits, i, 10, UtcParam.WNLSF & 0x3ff); i += 10;
	SetBits(Bits, i, 4, UtcParam.DN & 0xf); i += 4;
	SetSignedBits(Bits, i, 8, UtcParam.TLSF);
}

void NavICLNavBit::AppendCrc(int Bits[])
{
	unsigned int crc = 0;
	int i, bit;

	for (i = 0; i < NAVIC_LNAV_CRC_DATA_BITS + 2; i++)
	{
		bit = (i < 2) ? 0 : Bits[i - 2];
		crc ^= bit << 23;
		if (crc & 0x800000)
			crc = ((crc << 1) ^ 0x1864cfb) & 0xffffff;
		else
			crc = (crc << 1) & 0xffffff;
	}
	SetBits(Bits, NAVIC_LNAV_CRC_DATA_BITS, 24, crc & 0xffffff);
}

void NavICLNavBit::EncodeFrame(const int Bits[], int NavBits[])
{
	static const int Preamble[16] = {
		1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0
	};
	int ConvIn[NAVIC_LNAV_CONV_BITS];
	int Encoded[NAVIC_LNAV_SYMBOL_BITS];
	int i, j, r, g1, g2;

	for (i = 0; i < 16; i++)
		NavBits[i] = Preamble[i];

	for (i = 0; i < NAVIC_LNAV_CONV_BITS; i++)
		ConvIn[i] = (i < NAVIC_LNAV_DATA_BITS) ? Bits[i] : 0;

	r = 0;
	for (i = 0; i < NAVIC_LNAV_CONV_BITS; i++)
	{
		r = ((r << 1) | ConvIn[i]) & 0x7f;
		g1 = g2 = 0;
		for (j = 0; j < 7; j++)
		{
			if ((0x4f >> j) & 1)
				g1 ^= (r >> j) & 1;
			if ((0x6d >> j) & 1)
				g2 ^= (r >> j) & 1;
		}
		Encoded[i * 2] = g1;
		Encoded[i * 2 + 1] = g2;
	}

	for (i = 0; i < 73; i++)
	{
		for (j = 0; j < 8; j++)
			NavBits[16 + j * 73 + i] = Encoded[i * 8 + j];
	}
}

void NavICLNavBit::SetBits(int Bits[], int Pos, int Width, unsigned int Value)
{
	int i;

	for (i = 0; i < Width; i++)
		Bits[Pos + i] = (Value >> (Width - i - 1)) & 1;
}

void NavICLNavBit::SetSignedBits(int Bits[], int Pos, int Width, int Value)
{
	SetBits(Bits, Pos, Width, (unsigned int)Value & ((Width == 32) ? 0xffffffffu : ((1u << Width) - 1)));
}

int NavICLNavBit::ScaleSigned(double Value, int Power)
{
	return (int)((Value >= 0.0) ? (ldexp(Value, Power) + 0.5) : (ldexp(Value, Power) - 0.5));
}

unsigned int NavICLNavBit::ScaleUnsigned(double Value, int Power)
{
	return (unsigned int)(ldexp(Value, Power) + 0.5);
}

int NavICLNavBit::ScaleSemiCircleSigned(double Value, int Power)
{
	double SemiCircle = Value / PI;
	return ScaleSigned(SemiCircle, Power);
}

unsigned int NavICLNavBit::ScaleSemiCircleUnsigned(double Value, int Power)
{
	double SemiCircle = Value / PI;
	return ScaleUnsigned(SemiCircle, Power);
}
