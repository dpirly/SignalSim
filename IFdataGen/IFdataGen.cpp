#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <iostream>
#include <cstring>
#include <chrono>
#include <string>
#include <vector>
#include <ctime>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#ifdef _OPENMP
#include <omp.h>
#endif

#include "SignalSim.h"

#ifdef _MSC_VER
static int rand_r(unsigned int* seed)
{
	*seed = *seed * 1103515245u + 12345u;
	return (int)((*seed >> 16) & 0x7fff);
}

static int64_t GetFileOffset(FILE *File)
{
	return _ftelli64(File);
}
#else
static int64_t GetFileOffset(FILE *File)
{
	return ftello(File);
}
#endif

#define TOTAL_GPS_SAT 32
#define TOTAL_BDS_SAT 63
#define TOTAL_GAL_SAT 36
#define TOTAL_GLO_SAT 24
#define TOTAL_SAT_CHANNEL 128

typedef enum {
    DataBitLNav, DataBitCNav, DataBitCNav2, // for GPS
    DataBitGNav, DataBitGNav2,	// for GLONASS
    DataBitD1D2, DataBitBCNav1, DataBitBCNav2, DataBitBCNav3,	// for BDS
    DataBitINav, DataBitFNav, DataBitECNav,	// for Galileo
    DataBitSbas, // for SBAS
} DataBitType;

struct CommandArguments
{
	std::string ConfigFile;
	std::string OutputFile;
	bool MultiThread;
	bool ValidateOnly;
	bool OutputTag;
};

void UpdateSatParamList(GNSS_TIME CurTime, KINEMATIC_INFO CurPos, int ListCount, PSIGNAL_POWER PowerList, PIONO_PARAM IonoParam);
int StepToNextMs();
complex_number GenerateNoise(double Sigma);
complex_number GenerateNoise(unsigned int &Seed, double Sigma);
NavBit* GetNavData(GnssSystem SatSystem, int SatSignalIndex, NavBit* NavBitArray[]);
int QuantSamplesIQ2(complex_number Samples[], int Length, unsigned char QuantSamples[], double GainScale);	//TODO: Varify 2-bit quantization
int QuantSamplesIQ4(complex_number Samples[], int Length, unsigned char QuantSamples[], double GainScale);
int QuantSamplesIQ8(complex_number Samples[], int Length, unsigned char QuantSamples[], double GainScale);
int QuantSamplesIQ16(complex_number Samples[], int Length, unsigned char QuantSamples[], double GainScale);

void ShowHelp(const char* ProgramName);
bool ParseCommandLineArgs(int argc, char* argv[], CommandArguments &Arguments);
void CreateTagFile(const std::string& tagFilePath, const OUTPUT_PARAM& outputParam);
int RunLs3wOutput(JsonObject *RootObject, const CommandArguments &Arguments,
	UTC_TIME UtcTime, LLA_POSITION StartPos, LOCAL_SPEED StartVel);

CTrajectory Trajectory;
CPowerControl PowerControl;
CNavData NavData;
OUTPUT_PARAM OutputParam;
GNSS_TIME CurTime;
PGPS_EPHEMERIS GpsEph[TOTAL_GPS_SAT], GpsEphVisible[TOTAL_GPS_SAT];
PGPS_EPHEMERIS BdsEph[TOTAL_BDS_SAT], BdsEphVisible[TOTAL_BDS_SAT];
PGPS_EPHEMERIS GalEph[TOTAL_GAL_SAT], GalEphVisible[TOTAL_GAL_SAT];
PGLONASS_EPHEMERIS GloEph[TOTAL_GLO_SAT], GloEphVisible[TOTAL_GLO_SAT];
//SATELLITE_PARAM GpsSatParam[TOTAL_GPS_SAT], BdsSatParam[TOTAL_BDS_SAT], GalSatParam[TOTAL_GAL_SAT], GloSatParam[TOTAL_GLO_SAT];	// satellite parameter array at CurTime
CSatelliteParam GpsSatParam[TOTAL_GPS_SAT], BdsSatParam[TOTAL_BDS_SAT], GalSatParam[TOTAL_GAL_SAT], GloSatParam[TOTAL_GLO_SAT];
int GpsSatNumber, BdsSatNumber, GalSatNumber, GloSatNumber;	// number of visible satellite
const int SignalCenterFreq[][8] = {
	{ FREQ_GPS_L1, FREQ_GPS_L1, FREQ_GPS_L2, FREQ_GPS_L2, FREQ_GPS_L5 },
	{ FREQ_BDS_B1C, FREQ_BDS_B1I, FREQ_BDS_B2I, FREQ_BDS_B3I, FREQ_BDS_B2a, FREQ_BDS_B2b, FREQ_BDS_B2ab },
	{ FREQ_GAL_E1, FREQ_GAL_E5a, FREQ_GAL_E5b, FREQ_GAL_E5, FREQ_GAL_E6 },
	{ FREQ_GLO_G1, FREQ_GLO_G2 },
};
const char *SignalName[][8] = {
	{ "L1CA", "L1C", "L2C", "L2P", "L5", },
	{ "B1C", "B1I", "B2I", "B3I", "B2a", "B2b", "B2ab", },
	{ "E1", "E5a", "E5b", "E5", "E6", },
	{ "G1", "G2", },
};

int main(int argc, char* argv[])
{
	int i, j;
	JsonStream JsonTree;
	JsonObject *Object;
	UTC_TIME UtcTime;
	LLA_POSITION StartPos;
	KINEMATIC_INFO CurPos;
	LOCAL_SPEED StartVel;
	NavBit *NavBitArray[12];
	GLONASS_TIME GlonassTime;
	GNSS_TIME BdsTime;
	int ListCount;
	PSIGNAL_POWER PowerList;
	int FreqLow, FreqHigh;
	CSatIfSignal* SatIfSignal[TOTAL_SAT_CHANNEL];
	int TotalChannelNumber, SignalIndex;
	int IfFreq, FdmaOffset;
	complex_number *NoiseArray;
	unsigned char *QuantArray;
	FILE* IfFile = NULL;
	CommandArguments Arguments;

	// Default arguments
	Arguments.ConfigFile = "IfGenTest.json"; // Default JSON file
	Arguments.OutputFile = "";
	Arguments.MultiThread = true; // Default to use multi-threading
	Arguments.ValidateOnly = false;
	Arguments.OutputTag = false;

	SetOutputFile(stdout);
//	SetOutputLevel(MSG_LEVEL_INFO);

	// Show help if no arguments provided
	if (argc == 1)
	{
		ShowHelp(argv[0]);
		return 0;
	}

	if (!ParseCommandLineArgs(argc, argv, Arguments))
		return 1;

	
	printf("\n================================================================================\n");
	printf("                          IF SIGNAL GENERATION \n");
	printf("================================================================================\n");

    // Read the JSON file
	printf("[INFO]\tLoading JSON file: %s\n", Arguments.ConfigFile.c_str());
    if (JsonTree.ReadFile(Arguments.ConfigFile.c_str()) != 0)
    {
        std::cerr << "[ERROR]\tUnable to read JSON file: " << Arguments.ConfigFile << "\n";
        return 1;
    }
	else
	{
		printf("[INFO]\tJSON file read successfully: %s\n", Arguments.ConfigFile.c_str());
	}

    Object = JsonTree.GetRootObject();

	AssignParameters(Object, &UtcTime, &StartPos, &StartVel, &Trajectory, &NavData, &OutputParam, &PowerControl, NULL);

	if (!Arguments.OutputFile.empty())
	{
		// Override output filename
		strncpy(OutputParam.filename, Arguments.OutputFile.c_str(), 255);
		OutputParam.filename[255] = '\0';
		printf("[INFO]\tUsing output file from command line: %s\n", OutputParam.filename);
	}

	if (OutputParam.Format == OutputFormatLS3W || OutputParam.Format == OutputFormatWAVE)
		return RunLs3wOutput(Object, Arguments, UtcTime, StartPos, StartVel);

	// Validate configuration and exit if requested
/*	if (Arguments.ValidateOnly)
	{
		// TODO: Fully Implement Validation
		printf("[INFO]\tConfiguration validation To Be Implemented.\n");
		printf("[INFO]\tOutput file: %s\n", OutputParam.filename);
		printf("[INFO]\tSample frequency: %.2f MHz\n", OutputParam.SampleFreq / 1e3);
		printf("[INFO]\tCenter frequency: %.4f MHz\n", OutputParam.CenterFreq / 1e3);
		printf("[INFO]\tFormat: %s\n", (OutputParam.Format == OutputFormatIQ2) ? "IQ2" : (OutputParam.Format == OutputFormatIQ4) ? "IQ4" : "IQ8");
		return 0;
	}*/

	// initial variables
 	Trajectory.ResetTrajectoryTime();
	CurTime = UtcToGpsTime(UtcTime);
	GlonassTime = UtcToGlonassTime(UtcTime);
	BdsTime = UtcToBdsTime(UtcTime);
	CurPos = LlaToEcef(StartPos);
	SpeedLocalToEcef(StartPos, StartVel, CurPos);

	if (!Arguments.ValidateOnly)
	{
		printf("[INFO]\tOpening output file: %s\n", OutputParam.filename);
		if ((IfFile = fopen(OutputParam.filename, "wb")) == NULL)
		{
			printf("[ERROR]\tFailed to open output file: %s\n", OutputParam.filename);
			return 0;
		}
		printf("[INFO]\tOutput file opened successfully.\n");
	}

	if (Arguments.OutputTag)
	{
		std::string TagFileName = OutputParam.filename;
		TagFileName += ".tag";	// append .tag
		CreateTagFile(TagFileName, OutputParam);
	}

#ifdef _OPENMP
	if (Arguments.MultiThread)
	{
		printf("[INFO]\tOpenMP configured for PARALLEL execution (%d threads auto-detected)\n", omp_get_max_threads());
	}
	else
	{
		omp_set_num_threads(1);
		printf("[INFO]\tOpenMP configured for SERIAL execution (1 thread)\n");
	}
#else
	if (Arguments.MultiThread)
		printf("[WARNING]\tParallel execution requested but OpenMP not available - using sequential processing\n");
	else
		printf("[INFO]\tOpenMP not available - using sequential processing\n");
#endif

	for (i = 0; i < TOTAL_GPS_SAT; i ++)
		GpsSatParam[i].CN0 = (int)(PowerControl.InitCN0 * 100 + 0.5);
	for (i = 0; i < TOTAL_BDS_SAT; i ++)
		BdsSatParam[i].CN0 = (int)(PowerControl.InitCN0 * 100 + 0.5);
	for (i = 0; i < TOTAL_GAL_SAT; i ++)
		GalSatParam[i].CN0 = (int)(PowerControl.InitCN0 * 100 + 0.5);
	for (i = 0; i < TOTAL_GLO_SAT; i++)
		GloSatParam[i].CN0 = (int)(PowerControl.InitCN0 * 100 + 0.5);
	// create naviagtion bit instances
	for (i = 0; i < sizeof(NavBitArray) / sizeof(NavBit*); i++)
	{
		switch (i)
		{
		case DataBitLNav:   NavBitArray[i] = new LNavBit; break;
		case DataBitCNav:   NavBitArray[i] = new CNavBit; break;
		case DataBitCNav2:  NavBitArray[i] = new CNav2Bit; break;
		case DataBitGNav:   NavBitArray[i] = new GNavBit; break;
		case DataBitGNav2:  NavBitArray[i] = (NavBit*)0; break;
		case DataBitD1D2:   NavBitArray[i] = new D1D2NavBit; break;
		case DataBitBCNav1: NavBitArray[i] = new BCNav1Bit; break;
		case DataBitBCNav2: NavBitArray[i] = new BCNav2Bit; break;
		case DataBitBCNav3: NavBitArray[i] = new BCNav3Bit; break;
		case DataBitINav:   NavBitArray[i] = new INavBit; break;
		case DataBitFNav:   NavBitArray[i] = new FNavBit; break;
		case DataBitECNav:  NavBitArray[i] = (NavBit*)0; break;
		case DataBitSbas:   NavBitArray[i] = (NavBit*)0; break;
		default:            NavBitArray[i] = (NavBit*)0; break;
		}
	}

	// determine whether signal within IF band
	FreqLow = (OutputParam.CenterFreq - OutputParam.SampleFreq / 2) * 1000;
	FreqHigh = (OutputParam.CenterFreq + OutputParam.SampleFreq / 2) * 1000;
	if (OutputParam.FreqSelect[GpsSystem])
	{
		if ((OutputParam.FreqSelect[GpsSystem] & (1 << SIGNAL_INDEX_L1CA)) && (FREQ_GPS_L1 < FreqLow || FREQ_GPS_L1 > FreqHigh))
			OutputParam.FreqSelect[GpsSystem] &= ~(1 << SIGNAL_INDEX_L1CA);
		if ((OutputParam.FreqSelect[GpsSystem] & (1 << SIGNAL_INDEX_L1C)) && (FREQ_GPS_L1 < FreqLow || FREQ_GPS_L1 > FreqHigh))
			OutputParam.FreqSelect[GpsSystem] &= ~(1 << SIGNAL_INDEX_L1C);
		if ((OutputParam.FreqSelect[GpsSystem] & (1 << SIGNAL_INDEX_L2C)) && (FREQ_GPS_L2 < FreqLow || FREQ_GPS_L2 > FreqHigh))
			OutputParam.FreqSelect[GpsSystem] &= ~(1 << SIGNAL_INDEX_L2C);
		if ((OutputParam.FreqSelect[GpsSystem] & (1 << SIGNAL_INDEX_L2P)) && (FREQ_GPS_L2 < FreqLow || FREQ_GPS_L2 > FreqHigh))
			OutputParam.FreqSelect[GpsSystem] &= ~(1 << SIGNAL_INDEX_L2P);
		if ((OutputParam.FreqSelect[GpsSystem] & (1 << SIGNAL_INDEX_L5)) && (FREQ_GPS_L5 < FreqLow || FREQ_GPS_L5 > FreqHigh))
			OutputParam.FreqSelect[GpsSystem] &= ~(1 << SIGNAL_INDEX_L5);
	}
	if (OutputParam.FreqSelect[BdsSystem])
	{
		if ((OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B1C)) && (FREQ_BDS_B1C < FreqLow || FREQ_BDS_B1C > FreqHigh))
			OutputParam.FreqSelect[BdsSystem] &= ~(1 << SIGNAL_INDEX_B1C);
		if ((OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B1I)) && (FREQ_BDS_B1I < FreqLow || FREQ_BDS_B1I > FreqHigh))
			OutputParam.FreqSelect[BdsSystem] &= ~(1 << SIGNAL_INDEX_B1I);
		if ((OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B2I)) && (FREQ_BDS_B2I < FreqLow || FREQ_BDS_B2I > FreqHigh))
			OutputParam.FreqSelect[BdsSystem] &= ~(1 << SIGNAL_INDEX_B2I);
		if ((OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B3I)) && (FREQ_BDS_B3I < FreqLow || FREQ_BDS_B3I > FreqHigh))
			OutputParam.FreqSelect[BdsSystem] &= ~(1 << SIGNAL_INDEX_B3I);
		if ((OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B2a)) && (FREQ_BDS_B2a < FreqLow || FREQ_BDS_B2a > FreqHigh))
			OutputParam.FreqSelect[BdsSystem] &= ~(1 << SIGNAL_INDEX_B2a);
		if ((OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B2b)) && (FREQ_BDS_B2b < FreqLow || FREQ_BDS_B2b > FreqHigh))
			OutputParam.FreqSelect[BdsSystem] &= ~(1 << SIGNAL_INDEX_B2b);
	}
	if (OutputParam.FreqSelect[GalileoSystem])
	{
		if ((OutputParam.FreqSelect[GalileoSystem] & (1 << SIGNAL_INDEX_E1)) && (FREQ_GAL_E1 < FreqLow || FREQ_GAL_E1 > FreqHigh))
			OutputParam.FreqSelect[GalileoSystem] &= ~(1 << SIGNAL_INDEX_E1);
		if ((OutputParam.FreqSelect[GalileoSystem] & (1 << SIGNAL_INDEX_E5a)) && (FREQ_GAL_E5a < FreqLow || FREQ_GAL_E5a > FreqHigh))
			OutputParam.FreqSelect[GalileoSystem] &= ~(1 << SIGNAL_INDEX_E5a);
		if ((OutputParam.FreqSelect[GalileoSystem] & (1 << SIGNAL_INDEX_E5b)) && (FREQ_GAL_E5b < FreqLow || FREQ_GAL_E5b > FreqHigh))
			OutputParam.FreqSelect[GalileoSystem] &= ~(1 << SIGNAL_INDEX_E5b);
		if ((OutputParam.FreqSelect[GalileoSystem] & (1 << SIGNAL_INDEX_E6)) && (FREQ_GAL_E6 < FreqLow || FREQ_GAL_E6 > FreqHigh))
			OutputParam.FreqSelect[GalileoSystem] &= ~(1 << SIGNAL_INDEX_E6);
	}
	if (OutputParam.FreqSelect[GlonassSystem])
	{
		if ((OutputParam.FreqSelect[GlonassSystem] & (1 << SIGNAL_INDEX_G1)) && (FREQ_GLO_G1 < FreqLow || FREQ_GLO_G1 > FreqHigh))
			OutputParam.FreqSelect[GlonassSystem] &= ~(1 << SIGNAL_INDEX_G1);
		if ((OutputParam.FreqSelect[GlonassSystem] & (1 << SIGNAL_INDEX_G2)) && (FREQ_GLO_G2 < FreqLow || FREQ_GLO_G2 > FreqHigh))
			OutputParam.FreqSelect[GlonassSystem] &= ~(1 << SIGNAL_INDEX_G2);
	}

	// set Ionosphere and UTC parameter for different navigation data bit
	NavBitArray[DataBitLNav]->SetIonoUtc(NavData.GetGpsIono(), NavData.GetGpsUtcParam());
	NavBitArray[DataBitCNav]->SetIonoUtc(NavData.GetGpsIono(), NavData.GetGpsUtcParam());
	NavBitArray[DataBitCNav2]->SetIonoUtc(NavData.GetGpsIono(), NavData.GetGpsUtcParam());
	NavBitArray[DataBitD1D2]->SetIonoUtc(NavData.GetBdsIono(), NavData.GetBdsUtcParam());
	NavBitArray[DataBitINav]->SetIonoUtc(NavData.GetGalileoIono(), NavData.GetGalileoUtcParam());
	NavBitArray[DataBitFNav]->SetIonoUtc(NavData.GetGalileoIono(), NavData.GetGalileoUtcParam());
	// Find ephemeris match current time and fill in data to generate bit stream
	for (i = 1; i <= TOTAL_GPS_SAT; i ++)
	{
		GpsEph[i-1] = NavData.FindEphemeris(GpsSystem, CurTime, i);
		NavBitArray[DataBitLNav]->SetEphemeris(i, GpsEph[i - 1]);
		NavBitArray[DataBitCNav]->SetEphemeris(i, GpsEph[i - 1]);
		NavBitArray[DataBitCNav2]->SetEphemeris(i, GpsEph[i - 1]);
	}
	for (i = 1; i <= TOTAL_BDS_SAT; i ++)
	{
		BdsEph[i-1] = NavData.FindEphemeris(BdsSystem, BdsTime, i);
		NavBitArray[DataBitD1D2]->SetEphemeris(i, BdsEph[i - 1]);
		NavBitArray[DataBitBCNav1]->SetEphemeris(i, BdsEph[i - 1]);
		NavBitArray[DataBitBCNav2]->SetEphemeris(i, BdsEph[i - 1]);
		NavBitArray[DataBitBCNav3]->SetEphemeris(i, BdsEph[i - 1]);
	}
	for (i = 1; i <= TOTAL_GAL_SAT; i++)
	{
		GalEph[i - 1] = NavData.FindEphemeris(GalileoSystem, CurTime, i);
		NavBitArray[DataBitINav]->SetEphemeris(i, GalEph[i - 1]);
		NavBitArray[DataBitFNav]->SetEphemeris(i, GalEph[i - 1]);
	}
	for (i = 1; i <= TOTAL_GLO_SAT; i++)
	{
		GloEph[i - 1] = NavData.FindGloEphemeris(GlonassTime, i);
		NavBitArray[DataBitGNav]->SetEphemeris(i, (PGPS_EPHEMERIS)GloEph[i - 1]);
	}
	NavData.CompleteAlmanac(BdsSystem, UtcTime);
	NavData.CompleteAlmanac(GalileoSystem, UtcTime);
	NavBitArray[DataBitLNav]->SetAlmanac(NavData.GetGpsAlmanac());
	NavBitArray[DataBitCNav]->SetAlmanac(NavData.GetGpsAlmanac());
	NavBitArray[DataBitCNav2]->SetAlmanac(NavData.GetGpsAlmanac());
	NavBitArray[DataBitD1D2]->SetAlmanac(NavData.GetBdsAlmanac());
	NavBitArray[DataBitBCNav1]->SetAlmanac(NavData.GetBdsAlmanac());
	NavBitArray[DataBitBCNav2]->SetAlmanac(NavData.GetBdsAlmanac());
	NavBitArray[DataBitBCNav3]->SetAlmanac(NavData.GetBdsAlmanac());
	NavBitArray[DataBitINav]->SetAlmanac(NavData.GetGalileoAlmanac());
	NavBitArray[DataBitFNav]->SetAlmanac(NavData.GetGalileoAlmanac());
	NavBitArray[DataBitGNav]->SetAlmanac((PGPS_ALMANAC)NavData.GetGlonassAlmanac());

	// calculate visible satellite at start time and calculate satellite parameters
	GpsSatNumber = (OutputParam.FreqSelect[GpsSystem]) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, GpsSystem, GpsEph, TOTAL_GPS_SAT, GpsEphVisible) : 0;
	BdsSatNumber = (OutputParam.FreqSelect[BdsSystem]) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, BdsSystem, BdsEph, TOTAL_BDS_SAT, BdsEphVisible) : 0;
	GalSatNumber = (OutputParam.FreqSelect[GalileoSystem]) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, GalileoSystem, GalEph, TOTAL_GAL_SAT, GalEphVisible) : 0;
	GloSatNumber = (OutputParam.FreqSelect[GlonassSystem]) ? GetGlonassVisibleSatellite(CurPos, GlonassTime, OutputParam, GloEph, TOTAL_GLO_SAT, GloEphVisible) : 0;

	CIonoKlobuchar8 IonoModel(NavData.GetGpsIono());
	for (i = 0; i < TOTAL_GPS_SAT; i ++)
		GpsSatParam[i].Initialize(GpsSystem, GpsEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);
	for (i = 0; i < TOTAL_BDS_SAT; i ++)
		BdsSatParam[i].Initialize(BdsSystem, BdsEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);
	for (i = 0; i < TOTAL_GAL_SAT; i ++)
		GalSatParam[i].Initialize(GalileoSystem, GalEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);
	for (i = 0; i < TOTAL_GLO_SAT; i ++)
		GloSatParam[i].Initialize(GlonassSystem, (PGPS_EPHEMERIS)GloEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);

	ListCount = PowerControl.GetPowerControlList(0, PowerList);
	UpdateSatParamList(CurTime, CurPos, ListCount, PowerList, NavData.GetGpsIono());

	// create CSatIfSignal class for visible satellite, all other satellites clear pointer to NULL
	memset(SatIfSignal, 0, sizeof(SatIfSignal));
	TotalChannelNumber = 0;
	printf("[INFO]\tGenerating IF data with following satellite signals:\n\n");
	
	// Enhanced signal display with cleaner formatting
	printf("[INFO]\tEnabled Signals:\n");

	// GPS signals
	if (OutputParam.FreqSelect[GpsSystem]) {
		printf("\tGPS : [ ");
		if (OutputParam.FreqSelect[GpsSystem] & (1 << SIGNAL_INDEX_L1CA)) printf("L1CA ");
		if (OutputParam.FreqSelect[GpsSystem] & (1 << SIGNAL_INDEX_L1C)) printf("L1C ");
		if (OutputParam.FreqSelect[GpsSystem] & (1 << SIGNAL_INDEX_L2C)) printf("L2C ");
		if (OutputParam.FreqSelect[GpsSystem] & (1 << SIGNAL_INDEX_L2P)) printf("L2P ");
		if (OutputParam.FreqSelect[GpsSystem] & (1 << SIGNAL_INDEX_L5)) printf("L5 ");
		printf("]\n");
	}
	
	// BDS signals
	if (OutputParam.FreqSelect[BdsSystem]) {
		printf("\tBDS : [ ");
		if (OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B1C)) printf("B1C ");
		if (OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B1I)) printf("B1I ");
		if (OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B2I)) printf("B2I ");
		if (OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B2a)) printf("B2a ");
		if (OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B2b)) printf("B2b ");
		if (OutputParam.FreqSelect[BdsSystem] & (1 << SIGNAL_INDEX_B3I)) printf("B3I ");
		printf("]\n");
	}
	
	// Galileo signals
	if (OutputParam.FreqSelect[GalileoSystem]) {
		printf("\tGAL : [ ");
		if (OutputParam.FreqSelect[GalileoSystem] & (1 << SIGNAL_INDEX_E1)) printf("E1 ");
		if (OutputParam.FreqSelect[GalileoSystem] & (1 << SIGNAL_INDEX_E5a)) printf("E5a ");
		if (OutputParam.FreqSelect[GalileoSystem] & (1 << SIGNAL_INDEX_E5b)) printf("E5b ");
		if (OutputParam.FreqSelect[GalileoSystem] & (1 << SIGNAL_INDEX_E6)) printf("E6 ");
		printf("]\n");
	}
	
	// GLONASS signals
	if (OutputParam.FreqSelect[GlonassSystem]) {
		printf("\tGLO : [ ");
		if (OutputParam.FreqSelect[GlonassSystem] & (1 << SIGNAL_INDEX_G1)) printf("G1 ");
		if (OutputParam.FreqSelect[GlonassSystem] & (1 << SIGNAL_INDEX_G2)) printf("G2 ");
		printf("]\n");
	}
	printf("\n");
	// Count total signals per system
	int GpsSignalCount = 0, BdsSignalCount = 0, GalSignalCount = 0, GloSignalCount = 0;
	for (SignalIndex = SIGNAL_INDEX_L1CA; SignalIndex <= SIGNAL_INDEX_L5; SignalIndex++)
		if (OutputParam.FreqSelect[GpsSystem] & (1 << SignalIndex)) GpsSignalCount++;
	for (SignalIndex = SIGNAL_INDEX_B1C; SignalIndex <= SIGNAL_INDEX_B2b; SignalIndex++)
		if (OutputParam.FreqSelect[BdsSystem] & (1 << SignalIndex)) BdsSignalCount++;
	for (SignalIndex = SIGNAL_INDEX_E1; SignalIndex <= SIGNAL_INDEX_E6; SignalIndex++)
		if (OutputParam.FreqSelect[GalileoSystem] & (1 << SignalIndex)) GalSignalCount++;
	for (SignalIndex = SIGNAL_INDEX_G1; SignalIndex <= SIGNAL_INDEX_G2; SignalIndex++)
		if (OutputParam.FreqSelect[GlonassSystem] & (1 << SignalIndex)) GloSignalCount++;


	printf("Signals Summary Table:\n");
	
	// Enhanced constellation summary table
	printf("+---------------+-------------+--------------+------------------------------+\n");
	printf("| Constellation | Visible SVs | Signals / SV | Total Signals / Visible SVs  |\n");
	printf("+---------------+-------------+--------------+------------------------------+\n");
	printf("| GPS           | %-11d | %-12d | %-28d |\n", GpsSatNumber, GpsSignalCount, GpsSatNumber * GpsSignalCount);
	printf("| BeiDou        | %-11d | %-12d | %-28d |\n", BdsSatNumber, BdsSignalCount, BdsSatNumber * BdsSignalCount);
	printf("| Galileo       | %-11d | %-12d | %-28d |\n", GalSatNumber, GalSignalCount, GalSatNumber * GalSignalCount);
	printf("| GLONASS       | %-11d | %-12d | %-28d |\n", GloSatNumber, GloSignalCount, GloSatNumber * GloSignalCount);
	printf("+---------------+-------------+--------------+------------------------------+\n");
	
	int TotalVisibleSVs = GpsSatNumber + BdsSatNumber + GalSatNumber + GloSatNumber;
	int TotalChannels = GpsSatNumber * GpsSignalCount + BdsSatNumber * BdsSignalCount + GalSatNumber * GalSignalCount + GloSatNumber * GloSignalCount;
	printf("Total Visible SVs = %d, Total channels = %d\n\n", TotalVisibleSVs, TotalChannels);

	// Detailed satellite and signal information in compact table format
	for (SignalIndex = SIGNAL_INDEX_L1CA; SignalIndex <= SIGNAL_INDEX_L5; SignalIndex++)
	{
		if (!(OutputParam.FreqSelect[GpsSystem] & (1 << SignalIndex)))
			continue;
		IfFreq = SignalCenterFreq[0][SignalIndex] - OutputParam.CenterFreq * 1000;
		printf("GPS %s with IF %+dkHz:\n", SignalName[0][SignalIndex], IfFreq / 1000);
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		printf("| SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) |\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		int svCount = 0;
		for (i = 0; i < GpsSatNumber; i++)
		{
			if (TotalChannelNumber >= TOTAL_SAT_CHANNEL)
				break;
			if (!Arguments.ValidateOnly)
			{
				SatIfSignal[TotalChannelNumber] = new CSatIfSignal(OutputParam.SampleFreq, IfFreq, GpsSystem, SignalIndex, GpsEphVisible[i]->svid);
				SatIfSignal[TotalChannelNumber]->InitState(CurTime, &GpsSatParam[GpsEphVisible[i]->svid-1], GetNavData(GpsSystem, SignalIndex, NavBitArray));
			}
			TotalChannelNumber++;
			
			if (svCount % 4 == 0) printf("|");
			printf(" %02d | %+12d |", GpsEphVisible[i]->svid, (int)GpsSatParam[GpsEphVisible[i]->svid-1].GetDoppler(SignalIndex));
			svCount++;
			if (svCount % 4 == 0) printf("\n");
		}
		// Fill remaining columns if needed
		while (svCount % 4 != 0) {
			printf("    |              |");
			svCount++;
		}
		if (svCount > 0 && (svCount-1) % 4 == 3) printf("\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n\n");
	}
	
	for (SignalIndex = SIGNAL_INDEX_B1C; SignalIndex <= SIGNAL_INDEX_B2b; SignalIndex++)
	{
		if (!(OutputParam.FreqSelect[BdsSystem] & (1 << SignalIndex)))
			continue;
		IfFreq = SignalCenterFreq[1][SignalIndex] - OutputParam.CenterFreq * 1000;
		printf("BeiDou %s with IF %+dkHz:\n", SignalName[1][SignalIndex], IfFreq / 1000);
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		printf("| SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) |\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");

		int svCount = 0;
		for (i = 0; i < BdsSatNumber; i++)
		{
			if (TotalChannelNumber >= TOTAL_SAT_CHANNEL)
				break;
			if (!Arguments.ValidateOnly)
			{
				SatIfSignal[TotalChannelNumber] = new CSatIfSignal(OutputParam.SampleFreq, IfFreq, BdsSystem, SignalIndex, BdsEphVisible[i]->svid);
				SatIfSignal[TotalChannelNumber]->InitState(CurTime, &BdsSatParam[BdsEphVisible[i]->svid - 1], GetNavData(BdsSystem, SignalIndex, NavBitArray));
			}
			TotalChannelNumber++;
			
			if (svCount % 4 == 0) printf("|");
			printf(" %02d | %+12d |", BdsEphVisible[i]->svid, (int)BdsSatParam[BdsEphVisible[i]->svid-1].GetDoppler(SignalIndex));
			svCount++;
			if (svCount % 4 == 0) printf("\n");
		}
		while (svCount % 4 != 0) {
			printf("    |              |");
			svCount++;
		}
		if (svCount > 0 && (svCount-1) % 4 == 3) printf("\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n\n");
	}
	
	for (SignalIndex = SIGNAL_INDEX_E1; SignalIndex <= SIGNAL_INDEX_E6; SignalIndex++)
	{
		if (!(OutputParam.FreqSelect[GalileoSystem] & (1 << SignalIndex)))
			continue;
		IfFreq = SignalCenterFreq[2][SignalIndex] - OutputParam.CenterFreq * 1000;
		printf("Galileo %s with IF %+dkHz:\n", SignalName[2][SignalIndex], IfFreq / 1000);
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		printf("| SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) |\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		
		int svCount = 0;
		for (i = 0; i < GalSatNumber; i++)
		{
			if (TotalChannelNumber >= TOTAL_SAT_CHANNEL)
				break;
			if (!Arguments.ValidateOnly)
			{
				SatIfSignal[TotalChannelNumber] = new CSatIfSignal(OutputParam.SampleFreq, IfFreq, GalileoSystem, SignalIndex, GalEphVisible[i]->svid);
				SatIfSignal[TotalChannelNumber]->InitState(CurTime, &GalSatParam[GalEphVisible[i]->svid - 1], GetNavData(GalileoSystem, SignalIndex, NavBitArray));
			}
			TotalChannelNumber++;
			
			if (svCount % 4 == 0) printf("|");
			printf(" %02d | %+12d |", GalEphVisible[i]->svid, (int)GalSatParam[GalEphVisible[i]->svid-1].GetDoppler(SignalIndex));
			svCount++;
			if (svCount % 4 == 0) printf("\n");
		}
		while (svCount % 4 != 0) {
			printf("    |              |");
			svCount++;
		}
		if (svCount > 0 && (svCount-1) % 4 == 3) printf("\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n\n");
	}
	
	for (SignalIndex = SIGNAL_INDEX_G1; SignalIndex <= SIGNAL_INDEX_G2; SignalIndex++)
	{
		if (!(OutputParam.FreqSelect[GlonassSystem] & (1 << SignalIndex)))
			continue;
		IfFreq = SignalCenterFreq[3][SignalIndex] - OutputParam.CenterFreq * 1000;
		printf("GLONASS %s with IF %+dkHz:\n", SignalName[3][SignalIndex], IfFreq / 1000);
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		printf("| SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) |\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");

		int svCount = 0;
		for (i = 0; i < GloSatNumber; i++)
		{
			if (TotalChannelNumber >= TOTAL_SAT_CHANNEL)
				break;
			FdmaOffset = (SignalIndex == SIGNAL_INDEX_G1) ? GloEphVisible[i]->freq * 562500 : (SignalIndex == SIGNAL_INDEX_G2) ? GloEphVisible[i]->freq * 437500 : 0;
			if (!Arguments.ValidateOnly)
			{
				SatIfSignal[TotalChannelNumber] = new CSatIfSignal(OutputParam.SampleFreq, IfFreq + FdmaOffset, GlonassSystem, SignalIndex, GloEphVisible[i]->n);
				SatIfSignal[TotalChannelNumber]->InitState(CurTime, &GloSatParam[GloEphVisible[i]->n - 1], GetNavData(GlonassSystem, SignalIndex, NavBitArray));
			}
			TotalChannelNumber++;
			
			if (svCount % 4 == 0) printf("|");
			printf(" %02d | %+12d |", GloEphVisible[i]->n, (int)GloSatParam[GloEphVisible[i]->n-1].GetDoppler(SignalIndex));
			svCount++;
			if (svCount % 4 == 0) printf("\n");
		}
		while (svCount % 4 != 0) {
			printf("    |              |");
			svCount++;
		}
		if (svCount > 0 && (svCount-1) % 4 == 3) printf("\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n\n\n");

	}
	printf("Total channels: %d\n\n", TotalChannelNumber);

	int totalDurationMs = (int)(Trajectory.GetTimeLength() * 1000);
	double bytesPerMs = OutputParam.SampleFreq *  ((OutputParam.Format == OutputFormatIQ2) ? 0.5 : (OutputParam.Format == OutputFormatIQ4) ? 1.0: (OutputParam.Format == OutputFormatIQ16) ? 4.0 : 2.0);
	double totalMB = (totalDurationMs * bytesPerMs) / (1024.0 * 1024.0);
	printf("[INFO]\tSignal Duration: %0.2f s\n", totalDurationMs/1000.0);
	printf("[INFO]\tSignal Size: %.2f MB\n", totalMB);
	printf("[INFO]\tSignal Data format: %s\n", (OutputParam.Format == OutputFormatIQ2) ? "IQ2" : (OutputParam.Format == OutputFormatIQ4) ? "IQ4":(OutputParam.Format == OutputFormatIQ16) ? "IQ16" : "IQ8");
	printf("[INFO]\tSignal Center freq: %0.4f MHz\n", OutputParam.CenterFreq/1000.0);
	printf("[INFO]\tSignal Sample rate: %0.4f MHz\n\n", OutputParam.SampleFreq/1000.0);
	if (Arguments.ValidateOnly)
	{
		for (i = 0; i < static_cast<int>(sizeof(NavBitArray) / sizeof(NavBitArray[0])); ++i)
			delete NavBitArray[i];
		printf("Configuration validation completed\n");
		return 0;
	}

	NoiseArray = new complex_number[OutputParam.SampleFreq];
	QuantArray = new unsigned char[OutputParam.SampleFreq * 4];

	// Calculate total data size and setup progress tracking
	int exec_cycle = 0;
	long long TotalClippedSamples = 0;
	long long TotalSamples = 0;
	double AGCGain = 1.0;
	printf("[INFO]\tStarting signal generation loop...\n");
	fflush(stdout);
	
	auto start_time = std::chrono::high_resolution_clock::now();
	
	while (!StepToNextMs())
	{
		exec_cycle ++;
		// generate white noise
		for (i = 0; i < OutputParam.SampleFreq; i ++)
			NoiseArray[i] = GenerateNoise(1.0);
		
		// Use parallel or serial processing based on command line flag
		if (Arguments.MultiThread)
		{
			#ifdef _OPENMP
			// Parallel satellite signal generation using OpenMP (auto-detects thread count)
			#pragma omp parallel for schedule(dynamic)
			for (i = 0; i < TotalChannelNumber; i++)	// TOTAL_SAT_CHANNEL
				SatIfSignal[i]->GetIfSample(CurTime);

			#else
			// OpenMP not available, fall back to sequential processing
			for (i = 0; i < TotalChannelNumber; i++)
				SatIfSignal[i]->GetIfSample(CurTime);
			
			#endif
		}
		else
		{
			// True serial execution - no OpenMP overhead
			for (i = 0; i < TotalChannelNumber; i++)
				SatIfSignal[i]->GetIfSample(CurTime);

		}

		// Sequential accumulation to avoid race conditions (Dont nest this loop, causes issues with OpenMP)
		for (i = 0; i < TotalChannelNumber; i++)
		{
			for (j = 0; j < OutputParam.SampleFreq; j++)
				NoiseArray[j] += SatIfSignal[i]->SampleArray[j];
		}

		
		if (OutputParam.Format == OutputFormatIQ2) 
		{
			TotalClippedSamples += QuantSamplesIQ2(NoiseArray, OutputParam.SampleFreq, QuantArray, AGCGain);
			// Pack 2 samples per byte
			fwrite(QuantArray, sizeof(unsigned char), OutputParam.SampleFreq / 2, IfFile);	// 1/2 byte/sample
		}
		else if (OutputParam.Format == OutputFormatIQ4) 
		{
			TotalClippedSamples += QuantSamplesIQ4(NoiseArray, OutputParam.SampleFreq, QuantArray, AGCGain);
			fwrite(QuantArray, sizeof(unsigned char), OutputParam.SampleFreq, IfFile); // 1 byte/sample
		}
		else if (OutputParam.Format == OutputFormatIQ16) 
		{
			TotalClippedSamples += QuantSamplesIQ16(NoiseArray, OutputParam.SampleFreq, QuantArray, AGCGain);
			fwrite(QuantArray, sizeof(unsigned char) * 4, OutputParam.SampleFreq, IfFile); // 4 bytes/sample
		}
		else
		{
			TotalClippedSamples += QuantSamplesIQ8(NoiseArray, OutputParam.SampleFreq, QuantArray, AGCGain);
			fwrite(QuantArray, sizeof(unsigned char) * 2, OutputParam.SampleFreq, IfFile); // 2 bytes/sample
		}
		TotalSamples += OutputParam.SampleFreq * 2; // I and Q

#if 1
		// Adjust gain every 100ms
		if ((exec_cycle % 100) == 0)
		{
			double ClippingRate = (double)TotalClippedSamples / TotalSamples;
			if (ClippingRate > 0.01) // clipped rate over 1%
			{
				AGCGain *= 0.95; // reduce gain by 5%
				printf("[WARNING]\tAGC: Clipping %.2f%%, reducing gain to %.3f\n", ClippingRate * 100, AGCGain);
				TotalClippedSamples = TotalSamples = 0;	// reset statistic
			}
			else if (ClippingRate < 0.001 && AGCGain < 1.0) // clipped rate under 0.1%
			{
				AGCGain *= 1.02; // increase gain by 2%
				if (AGCGain > 1.0) AGCGain = 1.0;
				printf("[WARNING]\tAGC: Clipping %.2f%%, increasing gain to %.3f\n", ClippingRate * 100, AGCGain);
				TotalClippedSamples = TotalSamples = 0;	// reset statistic
			}
		}
#endif

//		for (j = 0; j < OutputParam.SampleFreq; j ++)
//			printf("%f %f\n", NoiseArray[j].real, NoiseArray[j].imag);
		// Enhanced progress reporting with percentage, MB/s, and ETA
		if ((exec_cycle % 25) == 0)
		{
			auto current_time = std::chrono::high_resolution_clock::now();
			auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
			
			double percentage = (double)exec_cycle / totalDurationMs * 100.0;
			double currentMB = (exec_cycle * bytesPerMs) / (1024.0 * 1024.0);
			double mbPerSec = (elapsed > 0) ? (currentMB * 1000.0) / elapsed : 0.0;
			
			// Calculate estimated time remaining
			long etaMs = 0;
			if (percentage > 0 && elapsed > 0) {
				etaMs = (long)((elapsed * (100.0 - percentage)) / percentage);
			}
			
			// Progress bar with percentage in center
			int barWidth = 50;
			int progress = (int)(percentage * barWidth / 100.0);
			char progressStr[8];
			sprintf(progressStr, "%.1f%%", percentage);
			int progressStrLen = strlen(progressStr);
			int centerPos = (barWidth - progressStrLen) / 2;
			
			printf("\r[");
			for (int k = 0; k < barWidth; k++) {
				if (k >= centerPos && k < centerPos + progressStrLen) {
					printf("%c", progressStr[k - centerPos]);
				} else if (k < progress) {
					printf("=");
				} else if (k == progress && percentage < 100.0) {
					printf(">");
				} else if (percentage >= 100.0 && k < barWidth) {
					printf("=");
				} else {
					printf(" ");
				}
			}
			
			// Format ETA
			char etaStr[32];
			if (etaMs > 0) {
				int etaSeconds = (int)(etaMs / 1000);
				int etaMinutes = etaSeconds / 60;
				etaSeconds %= 60;
				if (etaMinutes > 0) {
					sprintf(etaStr, "ETA: %dm%02ds   ", etaMinutes, etaSeconds);
				} else {
					sprintf(etaStr, "ETA: %02ds   ", etaSeconds);
				}
			} else {
				strcpy(etaStr, "ETA: --   ");
			}
			
			printf("] %d/%d ms | %.2f/%.2f MB @ %.2f MB/s | %s",
				   exec_cycle, totalDurationMs, currentMB, totalMB, mbPerSec, etaStr);
			fflush(stdout);
		}
//		if (length == 2) break;
	}
	
	// Final progress bar update to ensure 100% is shown
	printf("\r[");
	for (int k = 0; k < 50; k++) {
		if (k >= 22 && k < 28) {
			printf("%c", "100.0%"[k - 22]);
		} else {
			printf("=");
		}
	}
	printf("] %d/%d ms | %.2f/%.2f MB | \tCOMPLETED\n",
		   totalDurationMs, totalDurationMs, totalMB, totalMB); 
	
	auto end_time = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	double finalMB = (exec_cycle * bytesPerMs) / (1024.0 * 1024.0);
	double avgMbPerSec = (duration.count() > 0) ? (finalMB * 1000.0) / duration.count() : 0.0;

	printf("\n[INFO]\tIF Signal generation completed!\n");
	printf("------------------------------------------------------------------\n");
	printf("[INFO]\tTotal samples: %lld\n", TotalSamples);
	printf("[INFO]\tClipped samples: %lld (%.4f%%)\n", TotalClippedSamples, (double)TotalClippedSamples / TotalSamples * 100);
	printf("[INFO]\tFinal AGC gain: %.3f\n", AGCGain);
	if ((double)TotalClippedSamples / TotalSamples > 0.05)
	{
		printf("[WARNING]\tHigh clipping rate! Consider reducing initPower in JSON config.\n");
	}
	printf("[INFO]\tTotal time taken: %0.2f s\n", duration.count()/1000.0);
	printf("[INFO]\tData generated: %.2f MB\n", finalMB);
	printf("[INFO]\tAverage rate: %.2f MB/s\n", avgMbPerSec);
	printf("------------------------------------------------------------------\n\n");

	for (i = 0; i < TOTAL_SAT_CHANNEL; i ++)
		if (SatIfSignal[i]) delete SatIfSignal[i];
	for (i = 0; i < static_cast<int>(sizeof(NavBitArray) / sizeof(NavBitArray[0])); ++i)
		delete NavBitArray[i];
	delete[] NoiseArray;
	delete[] QuantArray;
	fclose(IfFile);

	return 0;
}

struct Ls3wPathConfig
{
	int CenterHz = 0;
	int BandwidthHz = 0;
	unsigned int FreqSelect[4] = {0, 0, 0, 0};
};

struct Ls3wPathRuntime
{
	Ls3wPathConfig Config;
	std::vector<CSatIfSignal*> Signals;
	std::vector<complex_number> Samples;
	unsigned int NoiseSeed = 1;
	int SignalCount[4] = {0, 0, 0, 0};
};

static JsonObject* FindJsonChild(JsonObject *Object, const char *Key)
{
	for (JsonObject *Child = Object ? Object->GetFirstObject() : NULL; Child; Child = Child->GetNextObject())
		if (strcmp(Child->Key, Key) == 0)
			return Child;
	return NULL;
}

static double JsonNumberValue(JsonObject *Object, double DefaultValue)
{
	if (!Object)
		return DefaultValue;
	if (Object->Type == JsonObject::ValueTypeIntNumber || Object->Type == JsonObject::ValueTypeFloatNumber)
		return GET_DOUBLE_VALUE(Object);
	return DefaultValue;
}

static int ParseSystemName(const char *Name)
{
	if (strcmp(Name, "GPS") == 0)
		return GpsSystem;
	if (strcmp(Name, "BDS") == 0 || strcmp(Name, "BeiDou") == 0)
		return BdsSystem;
	if (strcmp(Name, "Galileo") == 0 || strcmp(Name, "GAL") == 0)
		return GalileoSystem;
	if (strcmp(Name, "GLONASS") == 0 || strcmp(Name, "GLO") == 0)
		return GlonassSystem;
	return -1;
}

static int ParseSignalName(int System, const char *Name)
{
	int MaxSignal = 0;
	switch (System)
	{
	case GpsSystem: MaxSignal = SIGNAL_INDEX_L5; break;
	case BdsSystem: MaxSignal = SIGNAL_INDEX_B2ab; break;
	case GalileoSystem: MaxSignal = SIGNAL_INDEX_E6; break;
	case GlonassSystem: MaxSignal = SIGNAL_INDEX_G2; break;
	default: return -1;
	}
	for (int i = 0; i <= MaxSignal; ++i)
		if (SignalName[System][i] && strcmp(SignalName[System][i], Name) == 0)
			return i;
	return -1;
}

static void ParseLs3wSystemSelect(JsonObject *SelectArray, Ls3wPathConfig &Channel)
{
	if (!SelectArray || SelectArray->Type != JsonObject::ValueTypeArray)
		return;
	for (JsonObject *Item = SelectArray->GetFirstObject(); Item; Item = Item->GetNextObject())
	{
		JsonObject *SystemObject = FindJsonChild(Item, "system");
		JsonObject *SignalObject = FindJsonChild(Item, "signal");
		JsonObject *EnableObject = FindJsonChild(Item, "enable");
		if (!SystemObject || SystemObject->Type != JsonObject::ValueTypeString)
			continue;
		int System = ParseSystemName(SystemObject->String);
		if (System < 0 || System > GlonassSystem)
			continue;
		int Signal = 0;
		if (SignalObject && SignalObject->Type == JsonObject::ValueTypeString)
			Signal = ParseSignalName(System, SignalObject->String);
		if (Signal < 0)
			continue;
		bool Enable = !EnableObject || EnableObject->Type == JsonObject::ValueTypeTrue;
		if (Enable)
			Channel.FreqSelect[System] |= (1U << Signal);
		else
			Channel.FreqSelect[System] &= ~(1U << Signal);
	}
}

static bool ParseLs3wPaths(JsonObject *RootObject, std::vector<Ls3wPathConfig> &Channels)
{
	JsonObject *OutputObject = FindJsonChild(RootObject, "output");
	JsonObject *ChannelsObject = FindJsonChild(OutputObject, "ls3wPaths");
	if (!ChannelsObject || ChannelsObject->Type != JsonObject::ValueTypeArray)
		return false;
	for (JsonObject *Item = ChannelsObject->GetFirstObject(); Item; Item = Item->GetNextObject())
	{
		Ls3wPathConfig Channel;
		Channel.CenterHz = (int)(JsonNumberValue(FindJsonChild(Item, "centerFreq"), 0.0) * 1000000.0 + 0.5);
		Channel.BandwidthHz = (int)(JsonNumberValue(FindJsonChild(Item, "bandwidth"), 0.0) * 1000000.0 + 0.5);
		ParseLs3wSystemSelect(FindJsonChild(Item, "systemSelect"), Channel);
		if (Channel.CenterHz <= 0)
			return false;
		if (Channel.BandwidthHz <= 0)
			Channel.BandwidthHz = OutputParam.SampleFreq * 1000;
		Channels.push_back(Channel);
	}
	return !Channels.empty();
}

static int ParseLs3wQuantBits(JsonObject *RootObject)
{
	JsonObject *OutputObject = FindJsonChild(RootObject, "output");
	JsonObject *QuantObject = FindJsonChild(OutputObject, "quantBits");
	if (!QuantObject)
		QuantObject = FindJsonChild(FindJsonChild(OutputObject, "config"), "quantBits");
	int QuantBits = (int)JsonNumberValue(QuantObject, 1.0);
	return (QuantBits >= 1 && QuantBits <= 4) ? QuantBits : -1;
}

static void PutBitsMsb(uint64_t &Word, unsigned int BitOffset, unsigned int Value, unsigned int Width)
{
	for (unsigned int i = 0; i < Width; ++i)
		if ((Value >> (Width - 1U - i)) & 1U)
			Word |= 1ULL << (63U - (BitOffset + i));
}

static void WriteLe64(FILE *File, uint64_t Value)
{
	unsigned char Bytes[8];
	for (int i = 0; i < 8; ++i)
		Bytes[i] = (unsigned char)((Value >> (i * 8)) & 0xffU);
	fwrite(Bytes, 1, sizeof(Bytes), File);
}

static unsigned int QuantizeLs3w(double Value, int QuantBits)
{
	if (QuantBits == 1)
		return Value < 0.0 ? 1U : 0U;

	const unsigned int Half = 1U << (QuantBits - 1);
	const double Gain = 3.0;	// Match the original IQ4 quantizer scale.
	unsigned int Mag = (unsigned int)(fabs(Value) * Gain);
	if (Mag >= Half)
		Mag = Half - 1U;

	if (Value >= 0.0)
		return Mag;
	return (2U * Half - 1U) - Mag;
}

static std::string FormatDurationMs(long long DurationMs)
{
	long long Hours = DurationMs / 3600000;
	DurationMs %= 3600000;
	long long Minutes = DurationMs / 60000;
	DurationMs %= 60000;
	long long Seconds = DurationMs / 1000;
	long long Millis = DurationMs % 1000;
	char Buffer[64];
	snprintf(Buffer, sizeof(Buffer), "%02lld:%02lld:%02lld.%03lld", Hours, Minutes, Seconds, Millis);
	return Buffer;
}

static std::string MakeIniFileName(const char *OutputFileName)
{
	std::string IniFileName = OutputFileName;
	size_t DotPos = IniFileName.find_last_of('.');
	size_t SlashPos = IniFileName.find_last_of("/\\");
	if (DotPos != std::string::npos && (SlashPos == std::string::npos || DotPos > SlashPos))
		IniFileName.resize(DotPos);
	IniFileName += ".ini";
	return IniFileName;
}

static std::string BuildLs3wSignalList(const std::vector<Ls3wPathConfig> &Paths)
{
	std::string SignalList;
	for (size_t path = 0; path < Paths.size(); ++path)
	{
		for (int sys = GpsSystem; sys <= GlonassSystem; ++sys)
		{
			for (int sig = 0; sig < 8; ++sig)
			{
				if (!(Paths[path].FreqSelect[sys] & (1U << sig)))
					continue;
				std::string Signal = (sys == GpsSystem) ? "GPS_" :
					(sys == BdsSystem) ? "BDS_" :
					(sys == GalileoSystem) ? "GAL_" : "GLO_";
				Signal += SignalName[sys][sig];
				if (SignalList.find(Signal) == std::string::npos)
				{
					if (!SignalList.empty())
						SignalList += " ";
					SignalList += Signal;
				}
			}
		}
	}
	return SignalList;
}

static bool WriteLs3wIni(const char *OutputFileName, const std::vector<Ls3wPathConfig> &Paths, int QuantBits)
{
	std::string IniFileName = MakeIniFileName(OutputFileName);
	FILE *File = fopen(IniFileName.c_str(), "wb");
	if (!File)
	{
		printf("[ERROR]\tFailed to open ini file: %s\n", IniFileName.c_str());
		return false;
	}

	fprintf(File, "#LS3W config file\n\n");
	fprintf(File, "[config]\n");
	fprintf(File, "OSC=OCXO\n");
	fprintf(File, "SMP=%d\n", OutputParam.SampleFreq * 1000);
	fprintf(File, "QUA=%d\n", QuantBits);
	fprintf(File, "CHN=%zu\n", Paths.size());
	fprintf(File, "SFT=%zu\n", Paths.size() * QuantBits * 2);
	fprintf(File, "custom_profile=SignalSim\n\n");

	for (size_t i = 0; i < Paths.size(); ++i)
	{
		char Name = (char)('A' + i);
		fprintf(File, "[channel %c]\n", Name);
		fprintf(File, "CF%c=%d\n", Name, Paths[i].CenterHz);
		fprintf(File, "BW%c=%d\n", Name, Paths[i].BandwidthHz);
		fprintf(File, "\n");
	}

	fprintf(File, "[notes]\n");
	fprintf(File, "SRC=SignalSim\n");
	fprintf(File, "URL=https://github.com/globsky/SignalSim\n");
	fprintf(File, "AUTHOR=globsky\n");
	fprintf(File, "SIGNALS= %s\n", BuildLs3wSignalList(Paths).c_str());
	fclose(File);
	printf("[INFO]\tIni file created: %s\n", IniFileName.c_str());
	return true;
}

static std::string XmlEscape(const std::string &Text)
{
	std::string Escaped;
	for (size_t i = 0; i < Text.size(); ++i)
	{
		switch (Text[i])
		{
		case '&': Escaped += "&amp;"; break;
		case '<': Escaped += "&lt;"; break;
		case '>': Escaped += "&gt;"; break;
		case '"': Escaped += "&quot;"; break;
		case '\'': Escaped += "&apos;"; break;
		default: Escaped += Text[i]; break;
		}
	}
	return Escaped;
}

static std::string BuildWaveInfoXml(const std::vector<Ls3wPathConfig> &Paths, int QuantBits,
	long long SampleCount, const std::string &Duration, const char *DataName)
{
	char Buffer[256];
	std::string Xml = "<?xml version='1.0' encoding='UTF-8'?>\n";
	Xml += "<info version=\"1\" type=\"MCH\">\n";
	snprintf(Buffer, sizeof(Buffer), "  <SEGMENT_CLOCK>%d</SEGMENT_CLOCK>\n", OutputParam.SampleFreq * 1000);
	Xml += Buffer;
	Xml += "  <SEGMENT_COUNT>1</SEGMENT_COUNT>\n";
	snprintf(Buffer, sizeof(Buffer), "  <SEGMENT_LENGTH>%lld</SEGMENT_LENGTH>\n", SampleCount);
	Xml += Buffer;
	Xml += "  <SEGMENT_START>0</SEGMENT_START>\n";
	Xml += "  <DURATION>" + Duration + "</DURATION>\n";
	Xml += "  <WAVEFORM>" + XmlEscape(DataName) + "</WAVEFORM>\n";
	snprintf(Buffer, sizeof(Buffer), "  <QUANT_BITS>%d</QUANT_BITS>\n", QuantBits);
	Xml += Buffer;
	snprintf(Buffer, sizeof(Buffer), "  <CHANNEL_COUNT>%zu</CHANNEL_COUNT>\n", Paths.size());
	Xml += Buffer;
	snprintf(Buffer, sizeof(Buffer), "  <SAMPLE_FRAME_BITS>%zu</SAMPLE_FRAME_BITS>\n", Paths.size() * QuantBits * 2);
	Xml += Buffer;
	std::string SignalList = BuildLs3wSignalList(Paths);
	if (!SignalList.empty())
		Xml += "  <SIGNAL>" + XmlEscape(SignalList) + "</SIGNAL>\n";
	for (size_t i = 0; i < Paths.size(); ++i)
	{
		snprintf(Buffer, sizeof(Buffer), "  <CHANNEL index=\"%zu\">\n", i);
		Xml += Buffer;
		snprintf(Buffer, sizeof(Buffer), "    <CENTER>%d</CENTER>\n", Paths[i].CenterHz);
		Xml += Buffer;
		snprintf(Buffer, sizeof(Buffer), "    <BANDWIDTH>%d</BANDWIDTH>\n", Paths[i].BandwidthHz);
		Xml += Buffer;
		Xml += "  </CHANNEL>\n";
	}
	Xml += "</info>\n";
	return Xml;
}

static void WriteTarNumber(char *Field, size_t FieldSize, unsigned long long Value)
{
	memset(Field, 0, FieldSize);
	unsigned long long MaxOctal = 0;
	for (size_t i = 0; i < FieldSize - 1; ++i)
		MaxOctal = (MaxOctal << 3) | 7ULL;
	if (Value <= MaxOctal)
	{
		snprintf(Field, FieldSize, "%0*llo", (int)FieldSize - 1, Value);
		return;
	}

	for (size_t i = 0; i < FieldSize; ++i)
	{
		Field[FieldSize - 1 - i] = (char)(Value & 0xffU);
		Value >>= 8;
	}
	Field[0] |= (char)0x80;
}

static bool WriteTarHeader(FILE *File, const char *Name, uint64_t Size)
{
	if (strlen(Name) > 100)
	{
		printf("[ERROR]\tTar member name is too long: %s\n", Name);
		return false;
	}

	char Header[512];
	memset(Header, 0, sizeof(Header));
	memcpy(Header, Name, strlen(Name));
	WriteTarNumber(Header + 100, 8, 0644);
	WriteTarNumber(Header + 108, 8, 0);
	WriteTarNumber(Header + 116, 8, 0);
	WriteTarNumber(Header + 124, 12, Size);
	WriteTarNumber(Header + 136, 12, 0);
	memset(Header + 148, ' ', 8);
	Header[156] = '0';
	memcpy(Header + 257, "ustar", 5);
	memcpy(Header + 263, "00", 2);

	unsigned int Checksum = 0;
	for (size_t i = 0; i < sizeof(Header); ++i)
		Checksum += (unsigned char)Header[i];
	snprintf(Header + 148, 8, "%06o", Checksum);
	Header[154] = '\0';
	Header[155] = ' ';
	return fwrite(Header, 1, sizeof(Header), File) == sizeof(Header);
}

static bool WriteTarData(FILE *File, const void *Data, uint64_t Size)
{
	if (Size && fwrite(Data, 1, (size_t)Size, File) != Size)
		return false;
	uint64_t PadSize = (512 - (Size % 512)) % 512;
	if (PadSize)
	{
		char Pad[512] = {0};
		if (fwrite(Pad, 1, (size_t)PadSize, File) != PadSize)
			return false;
	}
	return true;
}

static bool FinishTarMember(FILE *File, uint64_t Size)
{
	uint64_t PadSize = (512 - (Size % 512)) % 512;
	if (PadSize)
	{
		char Pad[512] = {0};
		if (fwrite(Pad, 1, (size_t)PadSize, File) != PadSize)
			return false;
	}
	return true;
}

static bool FinishTarFile(FILE *File)
{
	char End[1024] = {0};
	return fwrite(End, 1, sizeof(End), File) == sizeof(End);
}

static void PrintLs3wPathSignalSelect(const Ls3wPathConfig &Channel)
{
	if (Channel.FreqSelect[GpsSystem])
	{
		printf("GPS : [ ");
		for (int SignalIndex = SIGNAL_INDEX_L1CA; SignalIndex <= SIGNAL_INDEX_L5; ++SignalIndex)
			if (Channel.FreqSelect[GpsSystem] & (1U << SignalIndex))
				printf("%s ", SignalName[GpsSystem][SignalIndex]);
		printf("] ");
	}
	if (Channel.FreqSelect[BdsSystem])
	{
		printf("BDS : [ ");
		for (int SignalIndex = SIGNAL_INDEX_B1C; SignalIndex <= SIGNAL_INDEX_B2ab; ++SignalIndex)
			if (Channel.FreqSelect[BdsSystem] & (1U << SignalIndex))
				printf("%s ", SignalName[BdsSystem][SignalIndex]);
		printf("] ");
	}
	if (Channel.FreqSelect[GalileoSystem])
	{
		printf("GAL : [ ");
		for (int SignalIndex = SIGNAL_INDEX_E1; SignalIndex <= SIGNAL_INDEX_E6; ++SignalIndex)
			if (Channel.FreqSelect[GalileoSystem] & (1U << SignalIndex))
				printf("%s ", SignalName[GalileoSystem][SignalIndex]);
		printf("] ");
	}
	if (Channel.FreqSelect[GlonassSystem])
	{
		printf("GLO : [ ");
		for (int SignalIndex = SIGNAL_INDEX_G1; SignalIndex <= SIGNAL_INDEX_G2; ++SignalIndex)
			if (Channel.FreqSelect[GlonassSystem] & (1U << SignalIndex))
				printf("%s ", SignalName[GlonassSystem][SignalIndex]);
		printf("] ");
	}
}

static void AppendSignalsForLs3wPath(Ls3wPathRuntime &Channel, size_t PathIndex, NavBit *NavBitArray[])
{
	for (int SignalIndex = SIGNAL_INDEX_L1CA; SignalIndex <= SIGNAL_INDEX_L5; ++SignalIndex)
	{
		if (!(Channel.Config.FreqSelect[GpsSystem] & (1U << SignalIndex)))
			continue;
		int IfFreq = SignalCenterFreq[GpsSystem][SignalIndex] - Channel.Config.CenterHz;
		Channel.SignalCount[GpsSystem] ++;
		printf("LS3W path %zu: GPS %s with IF %+dkHz:\n", PathIndex, SignalName[GpsSystem][SignalIndex], IfFreq / 1000);
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		printf("| SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) |\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		int svCount = 0;
		for (int i = 0; i < GpsSatNumber; ++i)
		{
			CSatIfSignal *Signal = new CSatIfSignal(OutputParam.SampleFreq, IfFreq, GpsSystem, SignalIndex, GpsEphVisible[i]->svid);
			Signal->InitState(CurTime, &GpsSatParam[GpsEphVisible[i]->svid - 1], GetNavData(GpsSystem, SignalIndex, NavBitArray));
			Channel.Signals.push_back(Signal);
			if (svCount % 4 == 0) printf("|");
			printf(" %02d | %+12d |", GpsEphVisible[i]->svid, (int)GpsSatParam[GpsEphVisible[i]->svid - 1].GetDoppler(SignalIndex));
			svCount++;
			if (svCount % 4 == 0) printf("\n");
		}
		while (svCount % 4 != 0) {
			printf("    |              |");
			svCount++;
		}
		if (svCount > 0 && (svCount - 1) % 4 == 3) printf("\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n\n");
	}
	for (int SignalIndex = SIGNAL_INDEX_B1C; SignalIndex <= SIGNAL_INDEX_B2ab; ++SignalIndex)
	{
		if (!(Channel.Config.FreqSelect[BdsSystem] & (1U << SignalIndex)))
			continue;
		int IfFreq = SignalCenterFreq[BdsSystem][SignalIndex] - Channel.Config.CenterHz;
		Channel.SignalCount[BdsSystem] ++;
		printf("LS3W path %zu: BeiDou %s with IF %+dkHz:\n", PathIndex, SignalName[BdsSystem][SignalIndex], IfFreq / 1000);
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		printf("| SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) |\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		int svCount = 0;
		for (int i = 0; i < BdsSatNumber; ++i)
		{
			CSatIfSignal *Signal = new CSatIfSignal(OutputParam.SampleFreq, IfFreq, BdsSystem, SignalIndex, BdsEphVisible[i]->svid);
			Signal->InitState(CurTime, &BdsSatParam[BdsEphVisible[i]->svid - 1], GetNavData(BdsSystem, SignalIndex, NavBitArray));
			Channel.Signals.push_back(Signal);
			if (svCount % 4 == 0) printf("|");
			printf(" %02d | %+12d |", BdsEphVisible[i]->svid, (int)BdsSatParam[BdsEphVisible[i]->svid - 1].GetDoppler(SignalIndex));
			svCount++;
			if (svCount % 4 == 0) printf("\n");
		}
		while (svCount % 4 != 0) {
			printf("    |              |");
			svCount++;
		}
		if (svCount > 0 && (svCount - 1) % 4 == 3) printf("\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n\n");
	}
	for (int SignalIndex = SIGNAL_INDEX_E1; SignalIndex <= SIGNAL_INDEX_E6; ++SignalIndex)
	{
		if (!(Channel.Config.FreqSelect[GalileoSystem] & (1U << SignalIndex)))
			continue;
		int IfFreq = SignalCenterFreq[GalileoSystem][SignalIndex] - Channel.Config.CenterHz;
		Channel.SignalCount[GalileoSystem] ++;
		printf("LS3W path %zu: Galileo %s with IF %+dkHz:\n", PathIndex, SignalName[GalileoSystem][SignalIndex], IfFreq / 1000);
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		printf("| SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) |\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		int svCount = 0;
		for (int i = 0; i < GalSatNumber; ++i)
		{
			CSatIfSignal *Signal = new CSatIfSignal(OutputParam.SampleFreq, IfFreq, GalileoSystem, SignalIndex, GalEphVisible[i]->svid);
			Signal->InitState(CurTime, &GalSatParam[GalEphVisible[i]->svid - 1], GetNavData(GalileoSystem, SignalIndex, NavBitArray));
			Channel.Signals.push_back(Signal);
			if (svCount % 4 == 0) printf("|");
			printf(" %02d | %+12d |", GalEphVisible[i]->svid, (int)GalSatParam[GalEphVisible[i]->svid - 1].GetDoppler(SignalIndex));
			svCount++;
			if (svCount % 4 == 0) printf("\n");
		}
		while (svCount % 4 != 0) {
			printf("    |              |");
			svCount++;
		}
		if (svCount > 0 && (svCount - 1) % 4 == 3) printf("\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n\n");
	}
	for (int SignalIndex = SIGNAL_INDEX_G1; SignalIndex <= SIGNAL_INDEX_G2; ++SignalIndex)
	{
		if (!(Channel.Config.FreqSelect[GlonassSystem] & (1U << SignalIndex)))
			continue;
		int IfFreq = SignalCenterFreq[GlonassSystem][SignalIndex] - Channel.Config.CenterHz;
		Channel.SignalCount[GlonassSystem] ++;
		printf("LS3W path %zu: GLONASS %s with IF %+dkHz:\n", PathIndex, SignalName[GlonassSystem][SignalIndex], IfFreq / 1000);
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		printf("| SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) | SV | Doppler (Hz) |\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n");
		int svCount = 0;
		for (int i = 0; i < GloSatNumber; ++i)
		{
			int FdmaOffset = (SignalIndex == SIGNAL_INDEX_G1) ? GloEphVisible[i]->freq * 562500 : GloEphVisible[i]->freq * 437500;
			CSatIfSignal *Signal = new CSatIfSignal(OutputParam.SampleFreq, IfFreq + FdmaOffset, GlonassSystem, SignalIndex, GloEphVisible[i]->n);
			Signal->InitState(CurTime, &GloSatParam[GloEphVisible[i]->n - 1], GetNavData(GlonassSystem, SignalIndex, NavBitArray));
			Channel.Signals.push_back(Signal);
			if (svCount % 4 == 0) printf("|");
			printf(" %02d | %+12d |", GloEphVisible[i]->n, (int)GloSatParam[GloEphVisible[i]->n - 1].GetDoppler(SignalIndex));
			svCount++;
			if (svCount % 4 == 0) printf("\n");
		}
		while (svCount % 4 != 0) {
			printf("    |              |");
			svCount++;
		}
		if (svCount > 0 && (svCount - 1) % 4 == 3) printf("\n");
		printf("+----+--------------+----+--------------+----+--------------+----+--------------+\n\n");
	}
}

int RunLs3wOutput(JsonObject *RootObject, const CommandArguments &Arguments,
	UTC_TIME UtcTime, LLA_POSITION StartPos, LOCAL_SPEED StartVel)
{
	std::vector<Ls3wPathConfig> Ls3wPathConfigs;
	bool WaveOutput = (OutputParam.Format == OutputFormatWAVE);
	int QuantBits = ParseLs3wQuantBits(RootObject);
	if (QuantBits < 0 || !ParseLs3wPaths(RootObject, Ls3wPathConfigs) || Ls3wPathConfigs.size() > 4)
	{
		printf("[ERROR]\tInvalid LS3W/WAVE output config: require quantBits=1..4 and 1..4 ls3wPaths\n");
		return 1;
	}
	if (OutputParam.SampleFreq <= 0 || OutputParam.filename[0] == 0)
	{
		printf("[ERROR]\tInvalid LS3W/WAVE output config: sampleFreq and name are required\n");
		return 1;
	}

	memset(OutputParam.FreqSelect, 0, sizeof(OutputParam.FreqSelect));
	for (size_t ch = 0; ch < Ls3wPathConfigs.size(); ++ch)
		for (int sys = 0; sys < 4; ++sys)
			OutputParam.FreqSelect[sys] |= Ls3wPathConfigs[ch].FreqSelect[sys];

	NavBit *NavBitArray[12];
	for (int i = 0; i < (int)(sizeof(NavBitArray) / sizeof(NavBitArray[0])); ++i)
	{
		switch (i)
		{
		case DataBitLNav:   NavBitArray[i] = new LNavBit; break;
		case DataBitCNav:   NavBitArray[i] = new CNavBit; break;
		case DataBitCNav2:  NavBitArray[i] = new CNav2Bit; break;
		case DataBitGNav:   NavBitArray[i] = new GNavBit; break;
		case DataBitD1D2:   NavBitArray[i] = new D1D2NavBit; break;
		case DataBitBCNav1: NavBitArray[i] = new BCNav1Bit; break;
		case DataBitBCNav2: NavBitArray[i] = new BCNav2Bit; break;
		case DataBitBCNav3: NavBitArray[i] = new BCNav3Bit; break;
		case DataBitINav:   NavBitArray[i] = new INavBit; break;
		case DataBitFNav:   NavBitArray[i] = new FNavBit; break;
		default:            NavBitArray[i] = (NavBit*)0; break;
		}
	}

	Trajectory.ResetTrajectoryTime();
	CurTime = UtcToGpsTime(UtcTime);
	GLONASS_TIME GlonassTime = UtcToGlonassTime(UtcTime);
	GNSS_TIME BdsTime = UtcToBdsTime(UtcTime);
	KINEMATIC_INFO CurPos = LlaToEcef(StartPos);
	SpeedLocalToEcef(StartPos, StartVel, CurPos);

	for (int i = 0; i < TOTAL_GPS_SAT; ++i)
		GpsSatParam[i].CN0 = (int)(PowerControl.InitCN0 * 100 + 0.5);
	for (int i = 0; i < TOTAL_BDS_SAT; ++i)
		BdsSatParam[i].CN0 = (int)(PowerControl.InitCN0 * 100 + 0.5);
	for (int i = 0; i < TOTAL_GAL_SAT; ++i)
		GalSatParam[i].CN0 = (int)(PowerControl.InitCN0 * 100 + 0.5);
	for (int i = 0; i < TOTAL_GLO_SAT; ++i)
		GloSatParam[i].CN0 = (int)(PowerControl.InitCN0 * 100 + 0.5);

	NavBitArray[DataBitLNav]->SetIonoUtc(NavData.GetGpsIono(), NavData.GetGpsUtcParam());
	NavBitArray[DataBitCNav]->SetIonoUtc(NavData.GetGpsIono(), NavData.GetGpsUtcParam());
	NavBitArray[DataBitCNav2]->SetIonoUtc(NavData.GetGpsIono(), NavData.GetGpsUtcParam());
	NavBitArray[DataBitD1D2]->SetIonoUtc(NavData.GetBdsIono(), NavData.GetBdsUtcParam());
	NavBitArray[DataBitINav]->SetIonoUtc(NavData.GetGalileoIono(), NavData.GetGalileoUtcParam());
	NavBitArray[DataBitFNav]->SetIonoUtc(NavData.GetGalileoIono(), NavData.GetGalileoUtcParam());

	for (int i = 1; i <= TOTAL_GPS_SAT; ++i)
	{
		GpsEph[i - 1] = NavData.FindEphemeris(GpsSystem, CurTime, i);
		NavBitArray[DataBitLNav]->SetEphemeris(i, GpsEph[i - 1]);
		NavBitArray[DataBitCNav]->SetEphemeris(i, GpsEph[i - 1]);
		NavBitArray[DataBitCNav2]->SetEphemeris(i, GpsEph[i - 1]);
	}
	for (int i = 1; i <= TOTAL_BDS_SAT; ++i)
	{
		BdsEph[i - 1] = NavData.FindEphemeris(BdsSystem, BdsTime, i);
		NavBitArray[DataBitD1D2]->SetEphemeris(i, BdsEph[i - 1]);
		NavBitArray[DataBitBCNav1]->SetEphemeris(i, BdsEph[i - 1]);
		NavBitArray[DataBitBCNav2]->SetEphemeris(i, BdsEph[i - 1]);
		NavBitArray[DataBitBCNav3]->SetEphemeris(i, BdsEph[i - 1]);
	}
	for (int i = 1; i <= TOTAL_GAL_SAT; ++i)
	{
		GalEph[i - 1] = NavData.FindEphemeris(GalileoSystem, CurTime, i);
		NavBitArray[DataBitINav]->SetEphemeris(i, GalEph[i - 1]);
		NavBitArray[DataBitFNav]->SetEphemeris(i, GalEph[i - 1]);
	}
	for (int i = 1; i <= TOTAL_GLO_SAT; ++i)
	{
		GloEph[i - 1] = NavData.FindGloEphemeris(GlonassTime, i);
		NavBitArray[DataBitGNav]->SetEphemeris(i, (PGPS_EPHEMERIS)GloEph[i - 1]);
	}
	NavData.CompleteAlmanac(BdsSystem, UtcTime);
	NavData.CompleteAlmanac(GalileoSystem, UtcTime);
	NavBitArray[DataBitLNav]->SetAlmanac(NavData.GetGpsAlmanac());
	NavBitArray[DataBitCNav]->SetAlmanac(NavData.GetGpsAlmanac());
	NavBitArray[DataBitCNav2]->SetAlmanac(NavData.GetGpsAlmanac());
	NavBitArray[DataBitD1D2]->SetAlmanac(NavData.GetBdsAlmanac());
	NavBitArray[DataBitBCNav1]->SetAlmanac(NavData.GetBdsAlmanac());
	NavBitArray[DataBitBCNav2]->SetAlmanac(NavData.GetBdsAlmanac());
	NavBitArray[DataBitBCNav3]->SetAlmanac(NavData.GetBdsAlmanac());
	NavBitArray[DataBitINav]->SetAlmanac(NavData.GetGalileoAlmanac());
	NavBitArray[DataBitFNav]->SetAlmanac(NavData.GetGalileoAlmanac());
	NavBitArray[DataBitGNav]->SetAlmanac((PGPS_ALMANAC)NavData.GetGlonassAlmanac());

	GpsSatNumber = (OutputParam.FreqSelect[GpsSystem]) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, GpsSystem, GpsEph, TOTAL_GPS_SAT, GpsEphVisible) : 0;
	BdsSatNumber = (OutputParam.FreqSelect[BdsSystem]) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, BdsSystem, BdsEph, TOTAL_BDS_SAT, BdsEphVisible) : 0;
	GalSatNumber = (OutputParam.FreqSelect[GalileoSystem]) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, GalileoSystem, GalEph, TOTAL_GAL_SAT, GalEphVisible) : 0;
	GloSatNumber = (OutputParam.FreqSelect[GlonassSystem]) ? GetGlonassVisibleSatellite(CurPos, GlonassTime, OutputParam, GloEph, TOTAL_GLO_SAT, GloEphVisible) : 0;

	CIonoKlobuchar8 IonoModel(NavData.GetGpsIono());
	for (int i = 0; i < TOTAL_GPS_SAT; ++i)
		GpsSatParam[i].Initialize(GpsSystem, GpsEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);
	for (int i = 0; i < TOTAL_BDS_SAT; ++i)
		BdsSatParam[i].Initialize(BdsSystem, BdsEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);
	for (int i = 0; i < TOTAL_GAL_SAT; ++i)
		GalSatParam[i].Initialize(GalileoSystem, GalEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);
	for (int i = 0; i < TOTAL_GLO_SAT; ++i)
		GloSatParam[i].Initialize(GlonassSystem, (PGPS_EPHEMERIS)GloEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);

	PSIGNAL_POWER PowerList = NULL;
	int ListCount = PowerControl.GetPowerControlList(0, PowerList);
	UpdateSatParamList(CurTime, CurPos, ListCount, PowerList, NavData.GetGpsIono());

	printf("[INFO]\tGenerating LS3W data with following satellite signals:\n\n");
	printf("[INFO]\tEnabled Signals:\n");
	for (size_t i = 0; i < Ls3wPathConfigs.size(); ++i)
	{
		printf("\tLS3W path %zu: [ center %.3f MHz, bandwidth %.3f MHz ] ",
			i, Ls3wPathConfigs[i].CenterHz / 1e6, Ls3wPathConfigs[i].BandwidthHz / 1e6);
		PrintLs3wPathSignalSelect(Ls3wPathConfigs[i]);
		printf("\n");
	}
	printf("\n");

	std::vector<Ls3wPathRuntime> Paths(Ls3wPathConfigs.size());
	for (size_t i = 0; i < Ls3wPathConfigs.size(); ++i)
	{
		Paths[i].Config = Ls3wPathConfigs[i];
		Paths[i].NoiseSeed = 1U + (unsigned int)i * 12345U;
		AppendSignalsForLs3wPath(Paths[i], i, NavBitArray);
		printf("[INFO]\tLS3W path %zu: total signal streams = %zu\n\n",
			i, Paths[i].Signals.size());
	}

	int DurationMs = (int)(Trajectory.GetTimeLength() * 1000 + 0.5);
	long long SampleCount = (long long)DurationMs * OutputParam.SampleFreq;
	unsigned int FrameBits = (unsigned int)(Paths.size() * QuantBits * 2);
	unsigned int SamplesPerWord = 64U / FrameBits;
	unsigned int SpareBits = 64U - SamplesPerWord * FrameBits;
	long long RegisterCount = (SampleCount + SamplesPerWord - 1) / SamplesPerWord;
	long long PackedSampleCount = RegisterCount * SamplesPerWord;
	uint64_t OutputBytes = (uint64_t)RegisterCount * 8U;
	std::string Duration = FormatDurationMs(DurationMs);
	double TotalMB = OutputBytes / (1024.0 * 1024.0);
	const char *FormatName = WaveOutput ? "WAVE" : "LS3W";

	for (size_t i = 0; i < Paths.size(); ++i)
		Paths[i].Samples.resize(OutputParam.SampleFreq);

	printf("[INFO]\tSignal Duration: %0.2f s (%s)\n", DurationMs / 1000.0, Duration.c_str());
	printf("[INFO]\tSignal Size: %.2f MB\n", TotalMB);
	printf("[INFO]\tSignal Data format: %s%d\n", FormatName, QuantBits);
	printf("[INFO]\tSignal Sample rate: %0.4f MHz\n", OutputParam.SampleFreq / 1000.0);
	printf("[INFO]\tLS3W paths: %zu, sample frame bits: %u, samples per 64-bit word: %u\n\n",
		Paths.size(), FrameBits, SamplesPerWord);

	if (Arguments.ValidateOnly)
	{
		for (size_t ch = 0; ch < Paths.size(); ++ch)
			for (size_t n = 0; n < Paths[ch].Signals.size(); ++n)
				delete Paths[ch].Signals[n];
		for (int i = 0; i < (int)(sizeof(NavBitArray) / sizeof(NavBitArray[0])); ++i)
			delete NavBitArray[i];
		printf("Configuration validation completed\n");
		return 0;
	}

	if (!WaveOutput && !WriteLs3wIni(OutputParam.filename, Ls3wPathConfigs, QuantBits))
		return 1;

	printf("[INFO]\tOpening output file: %s\n", OutputParam.filename);
	FILE *File = fopen(OutputParam.filename, "wb");
	if (!File)
	{
		printf("[ERROR]\tFailed to open output file: %s\n", OutputParam.filename);
		return 1;
	}
	printf("[INFO]\tOutput file opened successfully.\n");
	int64_t WaveDataStart = -1;
	if (WaveOutput)
	{
		std::string Xml = BuildWaveInfoXml(Ls3wPathConfigs, QuantBits, PackedSampleCount,
			Duration, "waveform.dat");
		if (!WriteTarHeader(File, "info.xml", Xml.size()) ||
			!WriteTarData(File, Xml.data(), Xml.size()) ||
			!WriteTarHeader(File, "waveform.dat", OutputBytes))
		{
			fclose(File);
			printf("[ERROR]\tFailed to write WAVE container header\n");
			return 1;
		}
		WaveDataStart = GetFileOffset(File);
		if (WaveDataStart < 0)
		{
			fclose(File);
			printf("[ERROR]\tFailed to get WAVE waveform data offset\n");
			return 1;
		}
		printf("[INFO]\tWAVE waveform.dat header size: %llu bytes\n",
			(unsigned long long)OutputBytes);
	}

#ifdef _OPENMP
	if (Arguments.MultiThread)
		printf("[INFO]\t%s generation uses OpenMP (%zu path worker threads + 1 writer thread, %d threads available)\n",
			FormatName, Paths.size(), omp_get_max_threads());
	else
		printf("[INFO]\t%s path generation uses single thread\n", FormatName);
#else
	if (Arguments.MultiThread)
		printf("[WARNING]\tParallel execution requested but OpenMP not available - using sequential processing\n");
	else
		printf("[INFO]\t%s path generation uses single thread\n", FormatName);
#endif
	printf("[INFO]\tStarting signal generation loop...\n");

	auto StartTime = std::chrono::high_resolution_clock::now();
	uint64_t Word = 0;
	unsigned int SamplesInWord = 0;
	int PathCount = (int)Paths.size();
	int MsGenerated = 0;
	int NextProgressMs = 1000;
#ifdef _OPENMP
	if (Arguments.MultiThread)
	{
		#pragma omp parallel num_threads(PathCount + 1)
		{
			int ThreadIndex = omp_get_thread_num();
			for (int ms = 0; ms < DurationMs; ++ms)
			{
				if (ThreadIndex == 0)
					StepToNextMs();
				#pragma omp barrier
				if (ThreadIndex > 0 && ThreadIndex <= PathCount)
				{
					int ch = ThreadIndex - 1;
					for (int s = 0; s < OutputParam.SampleFreq; ++s)
						Paths[ch].Samples[s] = complex_number(0.0, 0.0);
					for (size_t n = 0; n < Paths[ch].Signals.size(); ++n)
					{
						Paths[ch].Signals[n]->GetIfSample(CurTime);
						for (int s = 0; s < OutputParam.SampleFreq; ++s)
							Paths[ch].Samples[s] += Paths[ch].Signals[n]->SampleArray[s];
					}
				}
				#pragma omp barrier
				if (ThreadIndex == 0)
				{
					for (int s = 0; s < OutputParam.SampleFreq; ++s)
					{
						unsigned int BitPos = SpareBits + SamplesInWord * FrameBits;
						for (size_t ch = 0; ch < Paths.size(); ++ch)
						{
							PutBitsMsb(Word, BitPos, QuantizeLs3w(Paths[ch].Samples[s].real, QuantBits), QuantBits);
							BitPos += QuantBits;
							PutBitsMsb(Word, BitPos, QuantizeLs3w(Paths[ch].Samples[s].imag, QuantBits), QuantBits);
							BitPos += QuantBits;
						}
						if (++SamplesInWord == SamplesPerWord)
						{
							WriteLe64(File, Word);
							SamplesInWord = 0;
							Word = 0;
						}
					}
					MsGenerated = ms + 1;
					if (MsGenerated >= NextProgressMs || MsGenerated == DurationMs)
					{
						auto Now = std::chrono::high_resolution_clock::now();
						double Elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Now - StartTime).count() / 1000.0;
						double Percent = (double)MsGenerated * 100.0 / DurationMs;
						printf("\r[INFO]\t%s %.1f%% %d/%d ms %.2f MSps effective   ",
							FormatName, Percent, MsGenerated, DurationMs,
							Elapsed > 0 ? (MsGenerated * (double)OutputParam.SampleFreq) / (Elapsed * 1000000.0) : 0.0);
						fflush(stdout);
						while (NextProgressMs <= MsGenerated)
							NextProgressMs += 1000;
					}
				}
				#pragma omp barrier
			}
		}
	}
	else
#endif
	{
		for (int ms = 0; ms < DurationMs; ++ms)
		{
			StepToNextMs();
			for (int ch = 0; ch < PathCount; ++ch)
			{
				for (int s = 0; s < OutputParam.SampleFreq; ++s)
					Paths[ch].Samples[s] = complex_number(0.0, 0.0);
				for (size_t n = 0; n < Paths[ch].Signals.size(); ++n)
				{
					Paths[ch].Signals[n]->GetIfSample(CurTime);
					for (int s = 0; s < OutputParam.SampleFreq; ++s)
						Paths[ch].Samples[s] += Paths[ch].Signals[n]->SampleArray[s];
				}
			}
			for (int s = 0; s < OutputParam.SampleFreq; ++s)
			{
				unsigned int BitPos = SpareBits + SamplesInWord * FrameBits;
				for (size_t ch = 0; ch < Paths.size(); ++ch)
				{
					PutBitsMsb(Word, BitPos, QuantizeLs3w(Paths[ch].Samples[s].real, QuantBits), QuantBits);
					BitPos += QuantBits;
					PutBitsMsb(Word, BitPos, QuantizeLs3w(Paths[ch].Samples[s].imag, QuantBits), QuantBits);
					BitPos += QuantBits;
				}
				if (++SamplesInWord == SamplesPerWord)
				{
					WriteLe64(File, Word);
					SamplesInWord = 0;
					Word = 0;
				}
			}
			MsGenerated = ms + 1;
			if (MsGenerated >= NextProgressMs || MsGenerated == DurationMs)
			{
				auto Now = std::chrono::high_resolution_clock::now();
				double Elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Now - StartTime).count() / 1000.0;
				double Percent = (double)MsGenerated * 100.0 / DurationMs;
				printf("\r[INFO]\t%s %.1f%% %d/%d ms %.2f MSps effective   ",
					FormatName, Percent, MsGenerated, DurationMs,
					Elapsed > 0 ? (MsGenerated * (double)OutputParam.SampleFreq) / (Elapsed * 1000000.0) : 0.0);
				fflush(stdout);
				while (NextProgressMs <= MsGenerated)
					NextProgressMs += 1000;
			}
		}
	}
	if (SamplesInWord)
		WriteLe64(File, Word);
	if (WaveOutput)
	{
		int64_t WaveDataEnd = GetFileOffset(File);
		if (WaveDataEnd < 0)
		{
			fclose(File);
			printf("[ERROR]\tFailed to get WAVE waveform data end offset\n");
			return 1;
		}
		uint64_t ActualBytes = (uint64_t)(WaveDataEnd - WaveDataStart);
		if (ActualBytes != OutputBytes)
		{
			fclose(File);
			printf("[ERROR]\tWAVE waveform.dat size mismatch: header=%llu actual=%llu bytes\n",
				(unsigned long long)OutputBytes, (unsigned long long)ActualBytes);
			return 1;
		}
		printf("[INFO]\tWAVE waveform.dat actual size: %llu bytes\n",
			(unsigned long long)ActualBytes);
	}
	if (WaveOutput && (!FinishTarMember(File, OutputBytes) || !FinishTarFile(File)))
	{
		fclose(File);
		printf("[ERROR]\tFailed to finish WAVE container\n");
		return 1;
	}
	fclose(File);
	auto EndTime = std::chrono::high_resolution_clock::now();
	double Elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(EndTime - StartTime).count() / 1000.0;

	printf("\n\n[INFO]\t%s Signal generation completed!\n", FormatName);
	printf("------------------------------------------------------------------\n");
	printf("[INFO]\tTotal samples: %lld\n", SampleCount);
	printf("[INFO]\tData generated: %.2f MB\n", TotalMB);
	printf("[INFO]\tTotal time taken: %0.2f s\n", Elapsed);
	printf("[INFO]\tEffective sample rate: %.2f MSps\n",
		Elapsed > 0.0 ? (DurationMs * (double)OutputParam.SampleFreq) / (Elapsed * 1000000.0) : 0.0);
	printf("------------------------------------------------------------------\n\n");

	for (size_t ch = 0; ch < Paths.size(); ++ch)
		for (size_t n = 0; n < Paths[ch].Signals.size(); ++n)
			delete Paths[ch].Signals[n];
	for (int i = 0; i < (int)(sizeof(NavBitArray) / sizeof(NavBitArray[0])); ++i)
		delete NavBitArray[i];
	return 0;
}

void UpdateSatParamList(GNSS_TIME CurTime, KINEMATIC_INFO CurPos, int ListCount, PSIGNAL_POWER PowerList, PIONO_PARAM IonoParam)
{
	int i, index;
	LLA_POSITION PosLLA = EcefToLla(CurPos);
	int TotalSatNumber = 0;

	for (i = 0; i < GpsSatNumber; i ++)
	{
		index = GpsEphVisible[i]->svid - 1;
		GpsSatParam[index].CalculateParam(CurPos, PosLLA, CurTime);
		GpsSatParam[index].UpdateCN0(ListCount, PowerList);
	}
	for (i = 0; i < BdsSatNumber; i ++)
	{
		index = BdsEphVisible[i]->svid - 1;
		BdsSatParam[index].CalculateParam(CurPos, PosLLA, CurTime);
		BdsSatParam[index].UpdateCN0(ListCount, PowerList);
	}
	for (i = 0; i < GalSatNumber; i ++)
	{
		index = GalEphVisible[i]->svid - 1;
		GalSatParam[index].CalculateParam(CurPos, PosLLA, CurTime);
		GalSatParam[index].UpdateCN0(ListCount, PowerList);
	}
	for (i = 0; i < GloSatNumber; i++)
	{
		index = GloEphVisible[i]->n - 1;
		GloSatParam[index].CalculateParam(CurPos, PosLLA, CurTime);
		GloSatParam[index].UpdateCN0(ListCount, PowerList);
	}
}

int StepToNextMs()
{
	KINEMATIC_INFO CurPos;
	int ListCount = 0;
	PSIGNAL_POWER PowerList = NULL;
//	UTC_TIME UtcTime;
//	GLONASS_TIME GlonassTime;

	if (!Trajectory.GetNextPosVelECEF(0.001, CurPos))
		return -1;

	// calculate new satellite parameter
	ListCount = PowerControl.GetPowerControlList(1, PowerList);
	if (++CurTime.MilliSeconds > 604800000)
	{
		CurTime.Week ++;
		CurTime.MilliSeconds -= 604800000;
	}
/*	UtcTime = GpsTimeToUtc(CurTime);
	if ((CurTime.MilliSeconds % 60000) == 0)	// recalculate visible satellite at minute boundary
	{
		GlonassTime = UtcToGlonassTime(UtcTime);
		GpsSatNumber = (OutputParam.FreqSelect[GpsSystem]) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, GpsSystem, GpsEph, TOTAL_GPS_SAT, GpsEphVisible) : 0;
		BdsSatNumber = (OutputParam.FreqSelect[BdsSystem]) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, BdsSystem, BdsEph, TOTAL_BDS_SAT, BdsEphVisible) : 0;
		GalSatNumber = (OutputParam.FreqSelect[GalileoSystem]) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, GalileoSystem, GalEph, TOTAL_GAL_SAT, GalEphVisible) : 0;
		GloSatNumber = (OutputParam.FreqSelect[GlonassSystem]) ? GetGlonassVisibleSatellite(CurPos, GlonassTime, OutputParam, GloEph, TOTAL_GLO_SAT, GloEphVisible) : 0;
	}*/
	UpdateSatParamList(CurTime, CurPos, ListCount, PowerList, NavData.GetGpsIono());
	return 0;
}

complex_number GenerateNoise(double Sigma)
{
	double fvalue1, fvalue2, mag;

	// Marsaglia Polar method (improved Box-Muller method)
    do
	{
        fvalue1 = 2.0 * ((double)rand() / RAND_MAX) - 1.0;
        fvalue2 = 2.0 * ((double)rand() / RAND_MAX) - 1.0;
        mag = fvalue1 * fvalue1 + fvalue2 * fvalue2;
    } while (mag >= 1.0 || mag == 0.0);
	mag = sqrt(-2.0 * log(mag) / mag) * Sigma;

	return complex_number(fvalue1 * mag, fvalue2 * mag);
}

complex_number GenerateNoise(unsigned int &Seed, double Sigma)
{
	double fvalue1, fvalue2, mag;

	do
	{
		fvalue1 = 2.0 * ((double)rand_r(&Seed) / RAND_MAX) - 1.0;
		fvalue2 = 2.0 * ((double)rand_r(&Seed) / RAND_MAX) - 1.0;
		mag = fvalue1 * fvalue1 + fvalue2 * fvalue2;
	} while (mag >= 1.0 || mag == 0.0);
	mag = sqrt(-2.0 * log(mag) / mag) * Sigma;

	return complex_number(fvalue1 * mag, fvalue2 * mag);
}

NavBit* GetNavData(GnssSystem SatSystem, int SatSignalIndex, NavBit* NavBitArray[])
{
	switch (SatSystem)
	{
	case GpsSystem:
		switch (SatSignalIndex)
		{
		case SIGNAL_INDEX_L1CA: return NavBitArray[DataBitLNav];
		case SIGNAL_INDEX_L1C:  return NavBitArray[DataBitCNav2];
		case SIGNAL_INDEX_L2C:  return NavBitArray[DataBitCNav];
		case SIGNAL_INDEX_L2P:  return NavBitArray[DataBitLNav];
		case SIGNAL_INDEX_L5:   return NavBitArray[DataBitCNav];
		default: return NavBitArray[DataBitLNav];
		}
		break;
	case BdsSystem:
		switch (SatSignalIndex)
		{
		case SIGNAL_INDEX_B1C: return NavBitArray[DataBitBCNav1];
		case SIGNAL_INDEX_B1I: return NavBitArray[DataBitD1D2];
		case SIGNAL_INDEX_B2I: return NavBitArray[DataBitD1D2];
		case SIGNAL_INDEX_B3I: return NavBitArray[DataBitD1D2];
		case SIGNAL_INDEX_B2a: return NavBitArray[DataBitBCNav2];
		case SIGNAL_INDEX_B2b: return NavBitArray[DataBitBCNav3];
		default: return NavBitArray[DataBitD1D2];
		}
		break;
	case GalileoSystem:
		switch (SatSignalIndex)
		{
		case SIGNAL_INDEX_E1:  return NavBitArray[DataBitINav];
		case SIGNAL_INDEX_E5a: return NavBitArray[DataBitFNav];
		case SIGNAL_INDEX_E5b: return NavBitArray[DataBitINav];
		case SIGNAL_INDEX_E5:  return NavBitArray[DataBitFNav];
		case SIGNAL_INDEX_E6:  return NavBitArray[DataBitECNav];
		default: return NavBitArray[DataBitINav];
		}
		break;
	case GlonassSystem:
		switch (SatSignalIndex)
		{
		case SIGNAL_INDEX_G1: return NavBitArray[DataBitGNav];
		case SIGNAL_INDEX_G2: return NavBitArray[DataBitGNav];
		default: return NavBitArray[DataBitINav];
		}
		break;
	default: return NavBitArray[DataBitLNav];
	}
}

// PocketSDR compatible 2-bit IQ quantization 
// (TODO: Test)
// (FIXME: Optimize)
 int QuantSamplesIQ2(complex_number Samples[], int Length, unsigned char QuantSamples[], double GainScale)
 {
	int ClippedCount = 0;
	const double threshold = 1.1 / GainScale;	// the optimal threshold for Gauss noise is Sigma, increase a little to compensate added signal power
	const double ClippedThreshold = 5 * threshold;	// clip threshold set to 5 Sigma
	double Value;
	unsigned char QuantByte;

     // Process 2 complex samples at a time to produce 1 byte of output.
     // Bit definition within each byte is (from MSB): Sign-Q2, Mag-Q2, Sign-I2, Mag-I2, Sign-Q1, Mag-Q1, Sign-I1, Mag-I1
    for (int i = 0; i < Length; i += 2)
    {
		QuantByte = (Samples[i].real < 0.0) ? 2 : 0;
		Value = fabs(Samples[i].real);
		QuantByte |= (Value < threshold) ? 0 : 1;
		if (Value >= ClippedThreshold) ClippedCount ++;
		QuantByte |= (Samples[i].imag < 0.0) ? 8 : 0;
		Value = fabs(Samples[i].imag);
		QuantByte |= (Value < threshold) ? 0 : 4;
		if (Value >= ClippedThreshold) ClippedCount ++;

		QuantByte = (Samples[i+1].real < 0.0) ? 0x20 : 0;
		Value = fabs(Samples[i+1].real);
		QuantByte |= (Value < threshold) ? 0 : 0x10;
		if (Value >= ClippedThreshold) ClippedCount ++;
		QuantByte |= (Samples[i+1].imag < 0.0) ? 0x80 : 0;
		Value = fabs(Samples[i+1].imag);
		QuantByte |= (Value < threshold) ? 0 : 0x40;
		if (Value >= ClippedThreshold) ClippedCount ++;
	}

	return ClippedCount;
}

int QuantSamplesIQ4(complex_number Samples[], int Length, unsigned char QuantSamples[], double GainScale)
{
	int i;
	double Value;
	unsigned char QuantValue, QuantSample;
	const double Gain = GainScale * 3.0;
	int ClippedCount = 0;

	for (i = 0; i < Length; i++)
	{
		Value = fabs(Samples[i].real);
		QuantValue = (int)(Value * Gain);	// optimal quantization for sigma=1 noise
		if (QuantValue > 7)
		{
			QuantValue = 7;
			ClippedCount ++;
		}
		QuantValue += ((Samples[i].real >= 0) ? 0 : (1 << 3));	// add sign bit as MSB
		QuantSample = QuantValue << 4;
		Value = fabs(Samples[i].imag);
		QuantValue = (int)(Value * Gain);	// optimal quantization for sigma=1 noise
		if (QuantValue > 7)
		{
			QuantValue = 7;
			ClippedCount ++;
		}
		QuantValue += ((Samples[i].imag >= 0) ? 0 : (1 << 3));	// add sign bit as MSB
		QuantSample |= QuantValue;
		QuantSamples[i] = QuantSample;
	}

	return ClippedCount;
}

int QuantSamplesIQ8(complex_number Samples[], int Length, unsigned char QuantSamples[], double GainScale)
{
	int i;
	int QuantValue;
	const double Gain = GainScale * 25.0;
	int ClippedCount = 0;

	for (i = 0; i < Length; i++)
	{
		QuantValue = (int)(Samples[i].real * Gain);	// sigma scaled at 25 -> +-5 sigma scaled within range of INT8
		if (QuantValue > 127)	// saturate at -128~127
		{
			QuantValue = 127;
			ClippedCount ++;
		}
		else if (QuantValue < -128)
		{
			QuantValue = -128;
			ClippedCount ++;
		}
		QuantSamples[i * 2] = (unsigned char)(QuantValue & 0xff);
		QuantValue = (int)(Samples[i].imag * Gain);	// sigma scaled at 25 -> +-5 sigma scaled within range of INT8
		if (QuantValue > 127)	// saturate at -128~127
		{
			QuantValue = 127;
			ClippedCount ++;
		}
		else if (QuantValue < -128)
		{
			QuantValue = -128;
			ClippedCount ++;
		}
		QuantSamples[i * 2 + 1] = (unsigned char)(QuantValue & 0xff);
	}

	return ClippedCount;
}

int QuantSamplesIQ16(complex_number Samples[], int Length, unsigned char QuantSamples[], double GainScale)
{
    int i;
    int QuantValue;
    
    const double Gain = GainScale * 3277;
    int ClippedCount = 0;

    for (i = 0; i < Length; i++)
    {
        QuantValue = (int)(Samples[i].real * Gain);  
        if (QuantValue > 32767)  
        {
            QuantValue = 32767;
            ClippedCount++;
        }
        else if (QuantValue < -32768)
        {
            QuantValue = -32768;
            ClippedCount++;
        }
    
        QuantSamples[i * 4] = (unsigned char)(QuantValue & 0xff);       
        QuantSamples[i * 4 + 1] = (unsigned char)((QuantValue >> 8) & 0xff);  

      
        QuantValue = (int)(Samples[i].imag * Gain);  
        if (QuantValue > 32767)  
        {
            QuantValue = 32767;
            ClippedCount++;
        }
        else if (QuantValue < -32768)
        {
            QuantValue = -32768;
            ClippedCount++;
        }
      
        QuantSamples[i * 4 + 2] = (unsigned char)(QuantValue & 0xff);        
        QuantSamples[i * 4 + 3] = (unsigned char)((QuantValue >> 8) & 0xff);  
    }

    return ClippedCount;
}

void ShowHelp(const char* ProgramPath)
{
	// Extract just the executable name from the path
	std::string PathName = ProgramPath;
	std::string ProgramName;
    size_t pos = PathName.find_last_of("/\\");
    if (pos == std::string::npos)
		ProgramName = PathName;
	else
		ProgramName = PathName.substr(pos + 1);
	
	std::cout << "IFDataGen - GNSS IF Data Generator\n\n";
	std::cout << "Usage: " << ProgramName << " [options]\n\n";
	std::cout << "Available options:\n";
	std::cout << "   -c, 	--config <FILE>    Configuration file (JSON) [REQUIRED]\n";
	std::cout << "   -o, 	--output <FILE>    Output IF data file (overrides config)\n";
	std::cout << "   -vo, 	--validate-only    Validate configuration and exit\n";
	std::cout << "   -mt, 	--multi-thread     Force use multi-thread\n";
	std::cout << "   -st, 	--single-thread    Force use single-thread\n";
	std::cout << "   -t,  	--tag              Output tag file (output file name with .tag appended)\n";
	std::cout << "   -v, 	--version          Show version information\n";
	std::cout << "   -h, 	--help             Show this help message\n\n";
	std::cout << "Examples:\n";
	std::cout << "   " << ProgramName << " -c config.json\n";
	std::cout << "   " << ProgramName << " --config config.json --output mydata.bin\n";
	std::cout << "   " << ProgramName << " -c config.json -o output.bin -st\n";
	std::cout << "   " << ProgramName << " --config config.json -vo\n\n";
}

bool ParseCommandLineArgs(int argc, char* argv[], CommandArguments &Arguments)
{
	const std::vector<std::string> CommandList = {
		"--help", "-h",	// 0
		"--config", "-c",	// 1
		"--output", "-o",	// 2
		"--validate-only", "-vo",	// 3
		"--multi-thread", "-mt",	// 4
		"--single-thread", "-st",	// 5
		"--tag", "-t",	// 6
	};
	std::string arg;
	int i = 1, index;

	while (i < argc)
	{
		auto it = std::find(CommandList.begin(), CommandList.end(), argv[i]);
		index = (it == CommandList.end()) ? -1 : (it - CommandList.begin()) / 2;
		arg = argv[i];

		switch (index)
		{
		case 0:	// --help
			ShowHelp(argv[0]);
			exit(0);
		case 1:	// --config
			if (i + 1 >= argc || argv[i+1][0] == '-')
			{
				std::cerr << "[ERROR] " << arg << " requires a filename argument\n";
				return false;
			}
			Arguments.ConfigFile = argv[++i];
			break;
		case 2:	// --output
			if (i + 1 >= argc || argv[i+1][0] == '-')
			{
				std::cerr << "[ERROR] " << arg << " requires a filename argument\n";
				return false;
			}
			Arguments.OutputFile = argv[++i];
			break;
		case 3:	// --validate-only
			Arguments.ValidateOnly = true;
			break;
		case 4:	// --multi-thread
			Arguments.MultiThread = true;
			break;
		case 5:	// --single-thread
			Arguments.MultiThread = false;
			break;
		case 6:	// --tag
			Arguments.OutputTag = true;
			break;
		default:
			std::cout << "[WARNING] Unknown option " << arg << "\n";
		}
		i ++;	// move to next argument
	}

	return true;
}

void CreateTagFile(const std::string& tagFilePath, const OUTPUT_PARAM& outputParam)
{
    printf("[INFO]\tCreating tag file: %s\n", tagFilePath.c_str());
    FILE* tagFile = fopen(tagFilePath.c_str(), "w");
    
    if (!tagFile) {
        std::cerr << "[WARNING]\tCould not create tag file: " << tagFilePath << std::endl;
        return;
    }

    // Get current time in UTC (PocketSDR uses UTC time)
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    struct tm* timeinfo = gmtime(&time_t);  // Use gmtime() instead of localtime()
    
    // Determine format strings
    const char* formatStr;
    const char* iqStr;
    const char* bitsStr;

    if (outputParam.Format == OutputFormatIQ2) {
        formatStr = "INT2X2";
        iqStr = "1";
        bitsStr = "2";
    } else if (outputParam.Format == OutputFormatIQ4) {
        formatStr = "INT4X2";
        iqStr = "1";
        bitsStr = "4";
    }else if (outputParam.Format == OutputFormatIQ16){
	formatStr = "INT16X2";
        iqStr = "1";
        bitsStr = "16"; 
    }else { // OutputFormatIQ8
        formatStr = "INT8X2";
        iqStr = "1";
        bitsStr = "8";
    }

    // Write tag file contents matching PocketSDR format exactly
    fprintf(tagFile, "PROG = IFDataGen\n");
    fprintf(tagFile, "TIME = %04d/%02d/%02d %02d:%02d:%02d.%03d\n",
        timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
        timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, (int)ms.count());
    fprintf(tagFile, "FMT  = %s\n", formatStr);
    fprintf(tagFile, "F_S  = %.6f\n", outputParam.SampleFreq/1e3);		// Sample frequency in MHz
    fprintf(tagFile, "F_LO = %.6f\n", outputParam.CenterFreq/1e3);		// Center frequency in MHz
    fprintf(tagFile, "IQ   = %s\n", iqStr);
    fprintf(tagFile, "BITS = %s\n", bitsStr);
    fprintf(tagFile, "SCALE = 1.0\n");

    fclose(tagFile);
    printf("[INFO]\tTag file created: %s\n", tagFilePath.c_str());
}
