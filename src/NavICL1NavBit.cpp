//----------------------------------------------------------------------
// NavICL1NavBit.cpp:
//   Implementation of navigation bit synthesis class for NavIC L1-SPS NAV
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <math.h>
#include <memory.h>
#include <stdlib.h>

#include "ConstVal.h"
#include "NavICL1NavBit.h"

#define NAVIC_L1_SF1_SYMBOL_BITS 52
#define NAVIC_L1_SF2_DATA_BITS 576
#define NAVIC_L1_SF2_CODE_BITS 1200
#define NAVIC_L1_SF3_DATA_BITS 250
#define NAVIC_L1_SF3_CODE_BITS 548
#define NAVIC_L1_FRAME_BITS 1800
#define NAVIC_L1_AREF 42164200.0

static int Parity(unsigned int Value)
{
	Value ^= Value >> 16;
	Value ^= Value >> 8;
	Value ^= Value >> 4;
	Value ^= Value >> 2;
	Value ^= Value >> 1;
	return Value & 1;
}

NavICL1NavBit::NavICL1NavBit()
{
}

NavICL1NavBit::~NavICL1NavBit()
{
}

void NavICL1NavBit::SetBits(int Bits[], int Pos, int Width, uint64_t Value)
{
	for (int i = 0; i < Width; i++)
		Bits[Pos + i] = (int)((Value >> (Width - i - 1)) & 1);
}

void NavICL1NavBit::SetSignedBits(int Bits[], int Pos, int Width, int64_t Value)
{
	uint64_t Mask = (Width >= 64) ? UINT64_MAX : ((1ULL << Width) - 1ULL);

	SetBits(Bits, Pos, Width, (uint64_t)Value & Mask);
}

int64_t NavICL1NavBit::ScaleSigned(double Value, int Power)
{
	double Scaled = ldexp(Value, Power);

	return (int64_t)(Scaled >= 0.0 ? Scaled + 0.5 : Scaled - 0.5);
}

uint64_t NavICL1NavBit::ScaleUnsigned(double Value, int Power)
{
	double Scaled = ldexp(Value, Power);

	return (uint64_t)(Scaled + 0.5);
}

int64_t NavICL1NavBit::ScaleSemiCircleSigned(double Value, int Power)
{
	double SemiCircle = Value / PI;

	return ScaleSigned(SemiCircle, Power);
}

uint64_t NavICL1NavBit::ScaleSemiCircleUnsigned(double Value, int Power)
{
	double SemiCircle = Value / PI;

	return ScaleUnsigned(SemiCircle, Power);
}

int NavICL1NavBit::GetFrameData(GNSS_TIME StartTime, int svid, int Param, int *NavBits)
{
	int Sf2Bits[600], Sf3Bits[274];
	int Sf2Symbols[NAVIC_L1_SF2_CODE_BITS], Sf3Symbols[NAVIC_L1_SF3_CODE_BITS];
	int WeekMs, FrameCount, Toi, Itow;
	PGPS_EPHEMERIS Eph;

	if (!NavBits || svid < 1 || svid > 14)
		return 1;

	memset(NavBits, 0, sizeof(int) * NAVIC_L1_FRAME_BITS);
	memset(Sf2Bits, 0, sizeof(Sf2Bits));
	memset(Sf3Bits, 0, sizeof(Sf3Bits));

	WeekMs = StartTime.MilliSeconds;
	if (WeekMs < 0)
		WeekMs = (WeekMs % 604800000) + 604800000;
	WeekMs %= 604800000;

	FrameCount = WeekMs / 18000;
	Toi = (FrameCount + 1) % 400;
	Itow = (WeekMs / 1000) / 7200;

	ComposeSf1(Toi, NavBits);
	Eph = EphValid[svid - 1] ? (Ephemeris + svid - 1) : (PGPS_EPHEMERIS)0;
	if (Eph)
		ComposeSf2(Eph, Itow, Sf2Bits);
	else
	{
		SetBits(Sf2Bits, 0, 13, (StartTime.Week - 1024) & 0x1fff);
		SetBits(Sf2Bits, 13, 8, Itow & 0xff);
		SetBits(Sf2Bits, 21, 1, 1);
		for (int i = 22; i < NAVIC_L1_SF2_DATA_BITS; i++)
			Sf2Bits[i] = (i - 22) & 1;
	}
	ComposeSf3(FrameCount, Sf3Bits);
	AppendCrc(Sf2Bits, NAVIC_L1_SF2_DATA_BITS);
	AppendCrc(Sf3Bits, NAVIC_L1_SF3_DATA_BITS);

	if (EncodeLdpc(Sf2Bits, 600, Sf2Symbols) || EncodeLdpc(Sf3Bits, 274, Sf3Symbols))
		return 1;
	BlockInterleave(Sf2Symbols, Sf3Symbols, NavBits);

	return 0;
}

void NavICL1NavBit::ComposeSf1(int Toi, int NavBits[])
{
	unsigned int R = 0, Init = 0;

	for (int i = 0; i < 9; i++)
		Init = (Init << 1) | ((Toi >> i) & 1);
	R = Init;
	for (int i = 0; i < NAVIC_L1_SF1_SYMBOL_BITS; i++)
	{
		NavBits[i] = R & 1;
		R = (Parity(R & 0x1bf) << 8) | (R >> 1);
	}
}

void NavICL1NavBit::ComposeSf2(PGPS_EPHEMERIS Eph, int Itow, int Bits[])
{
	double Axis = Eph->sqrtA * Eph->sqrtA;
	int i = 0;

	SetBits(Bits, i, 13, (Eph->week - 1024) & 0x1fff); i += 13;
	SetBits(Bits, i, 8, Itow & 0xff); i += 8;
	SetBits(Bits, i, 1, 0); i += 1; // alert flag
	SetBits(Bits, i, 1, Eph->health ? 1 : 0); i += 1;
	SetBits(Bits, i, 4, Eph->iode & 0xf); i += 4;
	SetSignedBits(Bits, i, 5, Eph->ura); i += 5;
	SetBits(Bits, i, 11, (Eph->toe / 300) & 0x7ff); i += 11;
	SetSignedBits(Bits, i, 26, ScaleSigned(Axis - NAVIC_L1_AREF, 9)); i += 26;
	SetSignedBits(Bits, i, 26, ScaleSigned(Eph->axis_dot, 21)); i += 26;
	SetSignedBits(Bits, i, 19, ScaleSemiCircleSigned(Eph->delta_n, 44)); i += 19;
	SetSignedBits(Bits, i, 23, ScaleSemiCircleSigned(Eph->delta_n_dot, 57)); i += 23;
	SetSignedBits(Bits, i, 33, ScaleSemiCircleSigned(Eph->M0, 32)); i += 33;
	SetBits(Bits, i, 33, ScaleUnsigned(Eph->ecc, 34)); i += 33;
	SetSignedBits(Bits, i, 33, ScaleSemiCircleSigned(Eph->w, 32)); i += 33;
	SetSignedBits(Bits, i, 33, ScaleSemiCircleSigned(Eph->omega0, 32)); i += 33;
	SetSignedBits(Bits, i, 25, ScaleSemiCircleSigned(Eph->omega_dot, 44)); i += 25;
	SetSignedBits(Bits, i, 33, ScaleSemiCircleSigned(Eph->i0, 32)); i += 33;
	SetSignedBits(Bits, i, 15, ScaleSemiCircleSigned(Eph->idot, 44)); i += 15;
	SetSignedBits(Bits, i, 16, ScaleSigned(Eph->cis, 30)); i += 16;
	SetSignedBits(Bits, i, 16, ScaleSigned(Eph->cic, 30)); i += 16;
	SetSignedBits(Bits, i, 24, ScaleSigned(Eph->crs, 8)); i += 24;
	SetSignedBits(Bits, i, 24, ScaleSigned(Eph->crc, 8)); i += 24;
	SetSignedBits(Bits, i, 21, ScaleSigned(Eph->cus, 30)); i += 21;
	SetSignedBits(Bits, i, 21, ScaleSigned(Eph->cuc, 30)); i += 21;
	SetSignedBits(Bits, i, 29, ScaleSigned(Eph->af0, 35)); i += 29;
	SetSignedBits(Bits, i, 22, ScaleSigned(Eph->af1, 50)); i += 22;
	SetSignedBits(Bits, i, 15, ScaleSigned(Eph->af2, 66)); i += 15;
	SetSignedBits(Bits, i, 12, ScaleSigned(Eph->tgd, 35)); i += 12;
	SetSignedBits(Bits, i, 12, 0); i += 12;
	SetSignedBits(Bits, i, 12, 0); i += 12;
	SetBits(Bits, i, 1, 1); i += 1; // RSF: second delay field is ISCL1P
	SetBits(Bits, i, 3, 0);
}

void NavICL1NavBit::ComposeSf3(int FrameCount, int Bits[])
{
	switch (FrameCount % 6)
	{
	case 0:
		ComposeSf3Mt10(Bits);
		break;
	case 1:
		ComposeSf3Mt17(Bits);
		break;
	case 2:
		ComposeSf3Mt6(FrameCount, Bits);
		break;
	case 3:
		ComposeSf3Mt5(Bits);
		break;
	case 4:
		ComposeSf3Mt8(Bits);
		break;
	default:
		SetBits(Bits, 0, 6, 0); // null message
		SetBits(Bits, 6, 1, 1); // invalid data flag
		break;
	}
}

void NavICL1NavBit::ComposeSf3Mt5(int Bits[])
{
	int i = 7;

	SetBits(Bits, 0, 6, 5);
	SetBits(Bits, 6, 1, 1); // invalid until grid ionosphere data is available
	SetBits(Bits, i, 10, 0); i += 10; // regions masked
	SetBits(Bits, i, 4, 0); i += 4; // region id
	for (int n = 0; n < 15; n++)
	{
		SetBits(Bits, i, 4, 0); i += 4; // GIVEI
		SetSignedBits(Bits, i, 9, 0); i += 9; // GIVD
	}
	SetBits(Bits, i, 3, 0); i += 3; // IODI
	SetBits(Bits, i, 31, 0);
}

void NavICL1NavBit::ComposeSf3Mt6(int FrameCount, int Bits[])
{
	int Index = (FrameCount / 6) % 14;
	PGPS_ALMANAC Alm = (Almanac[Index].valid & 1) ? (Almanac + Index) : (PGPS_ALMANAC)0;
	int i = 7;

	SetBits(Bits, 0, 6, 6);
	SetBits(Bits, 6, 1, Alm ? 0 : 1); // invalid data flag
	if (!Alm)
		return;

	SetBits(Bits, i, 13, Alm->week & 0x1fff); i += 13;
	SetBits(Bits, i, 20, ScaleUnsigned(Alm->ecc, 21)); i += 20;
	SetBits(Bits, i, 16, (Alm->toa / 24) & 0xffff); i += 16;
	SetSignedBits(Bits, i, 24, ScaleSemiCircleSigned(Alm->i0, 23)); i += 24;
	SetSignedBits(Bits, i, 19, ScaleSemiCircleSigned(Alm->omega_dot, 38)); i += 19;
	SetBits(Bits, i, 24, ScaleUnsigned(Alm->sqrtA, 11)); i += 24;
	SetSignedBits(Bits, i, 24, ScaleSemiCircleSigned(Alm->omega0, 23)); i += 24;
	SetSignedBits(Bits, i, 24, ScaleSemiCircleSigned(Alm->w, 23)); i += 24;
	SetSignedBits(Bits, i, 24, ScaleSemiCircleSigned(Alm->M0, 23)); i += 24;
	SetSignedBits(Bits, i, 14, ScaleSigned(Alm->af0, 20)); i += 14;
	SetSignedBits(Bits, i, 11, ScaleSigned(Alm->af1, 38)); i += 11;
	SetBits(Bits, i, 6, Alm->svid & 0x3f); i += 6;
	SetBits(Bits, i, 24, 0);
}

void NavICL1NavBit::ComposeSf3Mt8(int Bits[])
{
	int i = 7;

	SetBits(Bits, 0, 6, 8);
	SetBits(Bits, 6, 1, 1); // invalid until NeQuick-N coefficients are available
	for (int n = 0; n < 3; n++)
	{
		SetSignedBits(Bits, i, 6, 0); i += 6; // MODIP max, scale 5 deg
		SetSignedBits(Bits, i, 6, 0); i += 6; // MODIP min, scale 5 deg
		SetSignedBits(Bits, i, 7, 0); i += 7; // longitude max, scale 5 deg
		SetSignedBits(Bits, i, 7, 0); i += 7; // longitude min, scale 5 deg
		SetBits(Bits, i, 11, 0); i += 11; // a0, scale 2^-2
		SetSignedBits(Bits, i, 11, 0); i += 11; // a1, scale 2^-8
		SetSignedBits(Bits, i, 14, 0); i += 14; // a2, scale 2^-15
		SetBits(Bits, i, 1, 0); i += 1; // IDF
	}
	SetBits(Bits, i, 3, 0); i += 3; // IODN
	SetBits(Bits, i, 51, 0);
}

void NavICL1NavBit::ComposeSf3Mt10(int Bits[])
{
	int i;

	SetBits(Bits, 0, 6, 10);
	SetBits(Bits, 6, 1, 0); // invalid data flag

	i = 7;
	SetBits(Bits, i, 16, 0); i += 16; // tEOP
	SetSignedBits(Bits, i, 21, 0); i += 21; // PM_X
	SetSignedBits(Bits, i, 15, 0); i += 15; // PM_X dot
	SetSignedBits(Bits, i, 21, 0); i += 21; // PM_Y
	SetSignedBits(Bits, i, 15, 0); i += 15; // PM_Y dot
	SetSignedBits(Bits, i, 31, 0); i += 31; // UT1-UTC
	SetSignedBits(Bits, i, 19, 0); i += 19; // UT1-UTC dot
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.a0, 30)); i += 8;
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.a1, 27)); i += 8;
	SetSignedBits(Bits, i, 10, ScaleSigned(IonoParam.a2, 24)); i += 10;
	SetSignedBits(Bits, i, 12, ScaleSigned(IonoParam.a3, 24)); i += 12;
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.b0, -11)); i += 8;
	SetSignedBits(Bits, i, 8, ScaleSigned(IonoParam.b1, -14)); i += 8;
	SetSignedBits(Bits, i, 11, ScaleSigned(IonoParam.b2, -16)); i += 11;
	SetSignedBits(Bits, i, 14, ScaleSigned(IonoParam.b3, -16)); i += 14;
	SetBits(Bits, i, 6, 18); i += 6; // max longitude 180 deg, 10 deg scale
	SetBits(Bits, i, 6, 0); i += 6; // min longitude 0 deg, 10 deg scale
	SetSignedBits(Bits, i, 5, 9); i += 5; // max latitude 90 deg, 10 deg scale
	SetSignedBits(Bits, i, 5, -9); i += 5; // min latitude -90 deg, 10 deg scale
	SetBits(Bits, i, 2, 0); i += 2; // IODK
	SetBits(Bits, i, 2, 0);
}

void NavICL1NavBit::ComposeSf3Mt17(int Bits[])
{
	int i;

	SetBits(Bits, 0, 6, 17);
	SetBits(Bits, 6, 1, 0); // invalid data flag

	i = 7;
	SetBits(Bits, i, 3, 0); i += 3; // IODT
	SetBits(Bits, i, 8, UtcParam.tot & 0xff); i += 8;
	SetBits(Bits, i, 13, UtcParam.WN & 0x1fff); i += 13;
	SetSignedBits(Bits, i, 8, UtcParam.TLS); i += 8;
	SetBits(Bits, i, 13, UtcParam.WNLSF & 0x1fff); i += 13;
	SetBits(Bits, i, 4, UtcParam.DN & 0xf); i += 4;
	SetSignedBits(Bits, i, 8, UtcParam.TLSF); i += 8;
	SetSignedBits(Bits, i, 16, ScaleSigned(UtcParam.A0, 35)); i += 16;
	SetSignedBits(Bits, i, 13, ScaleSigned(UtcParam.A1, 51)); i += 13;
	SetSignedBits(Bits, i, 7, ScaleSigned(UtcParam.A2, 68)); i += 7;
	SetBits(Bits, i, 1, 0); i += 1; // UTC(NPLI) invalid
	SetSignedBits(Bits, i, 16, 0); i += 16;
	SetSignedBits(Bits, i, 13, 0); i += 13;
	SetSignedBits(Bits, i, 7, 0); i += 7;

	for (int n = 0; n < 3; n++)
	{
		SetBits(Bits, i, 3, 7); i += 3;
		SetBits(Bits, i, 1, 0); i += 1;
		SetSignedBits(Bits, i, 16, 0); i += 16;
		SetSignedBits(Bits, i, 13, 0); i += 13;
	}
	SetBits(Bits, i, 14, 0);
}

void NavICL1NavBit::AppendCrc(int Bits[], int DataBits)
{
	unsigned int crc = 0;
	int AlignedBits = (DataBits + 7) / 8 * 8;
	int ZeroPadBits = AlignedBits - DataBits;
	int ByteCount = AlignedBits / 8;

	for (int i = 0; i < ByteCount; i++)
	{
		unsigned int data = 0;
		for (int j = 0; j < 8; j++)
		{
			int k = i * 8 + j - ZeroPadBits;
			data = (data << 1) | ((k >= 0 && Bits[k]) ? 1 : 0);
		}
		crc ^= data << 16;
		for (int j = 0; j < 8; j++)
		{
			if (crc & 0x800000)
				crc = ((crc << 1) ^ 0x1864cfb) & 0xffffff;
			else
				crc = (crc << 1) & 0xffffff;
		}
	}
	SetBits(Bits, DataBits, 24, crc & 0xffffff);
}

int NavICL1NavBit::EncodeLdpc(const int DataBits[], int DataLength, int CodeBits[])
{
	int *MatrixBits, *Rhs, *Parity;
	int Words = (DataLength + 31) / 32;
	int Result;

	memset(CodeBits, 0, sizeof(int) * DataLength * 2);
	MatrixBits = (int *)calloc(DataLength * Words, sizeof(int));
	Rhs = (int *)calloc(DataLength, sizeof(int));
	Parity = (int *)calloc(DataLength, sizeof(int));
	if (!MatrixBits || !Rhs || !Parity)
	{
		free(MatrixBits);
		free(Rhs);
		free(Parity);
		return 1;
	}

	if (DataLength == 600)
	{
		AddLdpcEdges(H_IRNV1_SF2_A, sizeof(H_IRNV1_SF2_A) / sizeof(H_IRNV1_SF2_A[0]),
			0, 0, DataBits, DataLength, MatrixBits, Rhs);
		AddLdpcEdges(H_IRNV1_SF2_B, sizeof(H_IRNV1_SF2_B) / sizeof(H_IRNV1_SF2_B[0]),
			0, DataLength, DataBits, DataLength, MatrixBits, Rhs);
		AddLdpcEdges(H_IRNV1_SF2_C, sizeof(H_IRNV1_SF2_C) / sizeof(H_IRNV1_SF2_C[0]),
			DataLength - 50, 0, DataBits, DataLength, MatrixBits, Rhs);
		AddLdpcEdges(H_IRNV1_SF2_D, sizeof(H_IRNV1_SF2_D) / sizeof(H_IRNV1_SF2_D[0]),
			DataLength - 50, DataLength, DataBits, DataLength, MatrixBits, Rhs);
		AddLdpcEdges(H_IRNV1_SF2_E, sizeof(H_IRNV1_SF2_E) / sizeof(H_IRNV1_SF2_E[0]),
			DataLength - 50, DataLength + 50, DataBits, DataLength, MatrixBits, Rhs);
		AddLdpcEdges(H_IRNV1_SF2_T, sizeof(H_IRNV1_SF2_T) / sizeof(H_IRNV1_SF2_T[0]),
			0, DataLength + 50, DataBits, DataLength, MatrixBits, Rhs);
	}
	else
	{
		AddLdpcEdges(H_IRNV1_SF3_A, sizeof(H_IRNV1_SF3_A) / sizeof(H_IRNV1_SF3_A[0]),
			0, 0, DataBits, DataLength, MatrixBits, Rhs);
		AddLdpcEdges(H_IRNV1_SF3_B, sizeof(H_IRNV1_SF3_B) / sizeof(H_IRNV1_SF3_B[0]),
			0, DataLength, DataBits, DataLength, MatrixBits, Rhs);
		AddLdpcEdges(H_IRNV1_SF3_C, sizeof(H_IRNV1_SF3_C) / sizeof(H_IRNV1_SF3_C[0]),
			DataLength - 23, 0, DataBits, DataLength, MatrixBits, Rhs);
		AddLdpcEdges(H_IRNV1_SF3_D, sizeof(H_IRNV1_SF3_D) / sizeof(H_IRNV1_SF3_D[0]),
			DataLength - 23, DataLength, DataBits, DataLength, MatrixBits, Rhs);
		AddLdpcEdges(H_IRNV1_SF3_E, sizeof(H_IRNV1_SF3_E) / sizeof(H_IRNV1_SF3_E[0]),
			DataLength - 23, DataLength + 23, DataBits, DataLength, MatrixBits, Rhs);
		AddLdpcEdges(H_IRNV1_SF3_T, sizeof(H_IRNV1_SF3_T) / sizeof(H_IRNV1_SF3_T[0]),
			0, DataLength + 23, DataBits, DataLength, MatrixBits, Rhs);
	}

	Result = SolveLdpc(DataLength, MatrixBits, Rhs, Parity);
	if (!Result)
	{
		for (int i = 0; i < DataLength; i++)
		{
			CodeBits[i] = DataBits[i] ? 1 : 0;
			CodeBits[DataLength + i] = Parity[i] ? 1 : 0;
		}
	}

	free(MatrixBits);
	free(Rhs);
	free(Parity);
	return Result;
}

void NavICL1NavBit::BlockInterleave(const int Sf2Symbols[], const int Sf3Symbols[], int NavBits[])
{
	int Symbols[1748];
	int k = 0;

	for (int i = 0; i < NAVIC_L1_SF2_CODE_BITS; i++)
		Symbols[k++] = Sf2Symbols[i];
	for (int i = 0; i < NAVIC_L1_SF3_CODE_BITS; i++)
		Symbols[k++] = Sf3Symbols[i];
	for (int row = 0, p = 0; row < 38; row++)
		for (int col = 0; col < 46; col++)
			NavBits[52 + col * 38 + row] = Symbols[p++];
}

void NavICL1NavBit::AddLdpcEdges(const NavICL1LdpcEntry Entries[], int Count, int RowOffset, int ColOffset,
	const int DataBits[], int DataLength, int MatrixBits[], int Rhs[])
{
	int Words = (DataLength + 31) / 32;

	for (int i = 0; i < Count; i++)
	{
		int Row = RowOffset + Entries[i].row - 1;
		int Col = ColOffset + Entries[i].col - 1;
		if (Col < DataLength)
			Rhs[Row] ^= DataBits[Col] ? 1 : 0;
		else
		{
			int Pcol = Col - DataLength;
			MatrixBits[Row * Words + Pcol / 32] ^= (1U << (Pcol & 31));
		}
	}
}

int NavICL1NavBit::SolveLdpc(int Rows, int MatrixBits[], int Rhs[], int Solution[])
{
	int Words = (Rows + 31) / 32;

	for (int col = 0; col < Rows; col++)
	{
		int Pivot = -1;
		for (int row = col; row < Rows; row++)
		{
			if (MatrixBits[row * Words + col / 32] & (1U << (col & 31)))
			{
				Pivot = row;
				break;
			}
		}
		if (Pivot < 0)
			return 1;
		if (Pivot != col)
		{
			for (int w = 0; w < Words; w++)
			{
				int Tmp = MatrixBits[col * Words + w];
				MatrixBits[col * Words + w] = MatrixBits[Pivot * Words + w];
				MatrixBits[Pivot * Words + w] = Tmp;
			}
			int T = Rhs[col];
			Rhs[col] = Rhs[Pivot];
			Rhs[Pivot] = T;
		}
		for (int row = 0; row < Rows; row++)
		{
			if (row == col || !(MatrixBits[row * Words + col / 32] & (1U << (col & 31))))
				continue;
			for (int w = 0; w < Words; w++)
				MatrixBits[row * Words + w] ^= MatrixBits[col * Words + w];
			Rhs[row] ^= Rhs[col];
		}
	}

	for (int i = 0; i < Rows; i++)
		Solution[i] = Rhs[i] & 1;
	return 0;
}
