#ifndef GM2_FIELD_ROOT_TREE_STRUCTS_HH_
#define GM2_FIELD_ROOT_TREE_STRUCTS_HH_

/*===========================================================================*\

author: Ran Hong
email:  rhong@anl.gov

about:  A header file for root-format output data file

\*===========================================================================*/

//--- project includes  -----------------------------------------------------//
#include <vector>
#include <string>
//--- project includes  -----------------------------------------------------//
//#include "gm2dataproducts/field/daq/field_constants.hh"
//#include "gm2field/include/FixedProbeGeometry.hh"

#include "Rtypes.h"
#include "RtypesCore.h"

#define NMR_NUM_FIXED_PROBES 378
#define TROLLEY_NUM_CH 17
#define TROLLEY_MULTIPOLE_NUM 9
#define SC_NUM_COILS 100
#define AZ_NUM_COILS 4
#define FREQ_METHOD_NUM 5
#define TROLLEY_NUM_CH 17
#define TROLLEY_MULTIPOLE_NUM 9
#define SC_NUM_COILS 100
#define SC_NUM_AZ_COILS 4
#define PROFILE_LEN 100
#define FLUXGATE_NUM_CH 24

#define SUM_VAR(type, name) type name ## Min; type name ## Max; type name ## Mean; type name ## RMS;
#define SUM_STR(name, type) name ## Min/ ## type ## : ## name ## Max/ ## type ## : ## name ## Mean/ ## type ## : ## name ## RMS/ ## type
#define SUM_VAR_ARRAY(type, name, N) type name ## Min[ N ]; type name ## Max [ N ]; type name ## Mean [ N ]; type name ## RMS [ N ];

float testvar = 10;

namespace gm2field {

  //Tree: fixedProbe
  //Branch: Frequency
  struct fixedProbeFrequency_t{
    ULong64_t SystemTimeStamp[NMR_NUM_FIXED_PROBES];
    ULong64_t GpsTimeStamp[NMR_NUM_FIXED_PROBES];
    ULong64_t DeviceTimeStamp[NMR_NUM_FIXED_PROBES];
    Double_t Frequency[NMR_NUM_FIXED_PROBES][FREQ_METHOD_NUM];
    Double_t FrequencyUncertainty[NMR_NUM_FIXED_PROBES][FREQ_METHOD_NUM];
//    Double_t DAQFrequency[NMR_NUM_FIXED_PROBES][FREQ_METHOD_NUM];
//    Double_t DAQFrequencyUncertainty[NMR_NUM_FIXED_PROBES][FREQ_METHOD_NUM];
  };

  const char * const fixedProbeFrequency_str = "SystemTimeStamp[378]/l:GpsTimeStamp[378]:DeviceTimeStamp[378]:Frequency[378][5]/D:FrequencyUncertainty[378][5]";

  struct fixedProbeSignal_t{
    Double_t Amplitude[NMR_NUM_FIXED_PROBES];
    Double_t Noise[NMR_NUM_FIXED_PROBES];
    Double_t FidLength[NMR_NUM_FIXED_PROBES];
    Double_t FidPower[NMR_NUM_FIXED_PROBES];
  };

  const char * const fixedProbeSignal_str = "Amplitude[378]/D:Noise[378]:FidLength[378]:FidPower[378]";

  struct fixedProbeTrigger_t{
    UInt_t TimeStamp[NMR_NUM_FIXED_PROBES];
    UShort_t Delay[NMR_NUM_FIXED_PROBES];
    UShort_t BunchInterval[NMR_NUM_FIXED_PROBES];
    UShort_t ReadoutTime[NMR_NUM_FIXED_PROBES];
//    UShort_t T0Time[NMR_NUM_FIXED_PROBES];
    UShort_t SequenceId[NMR_NUM_FIXED_PROBES];
    UShort_t BunchId[NMR_NUM_FIXED_PROBES];
    UShort_t Mode[NMR_NUM_FIXED_PROBES];
    UShort_t Source[NMR_NUM_FIXED_PROBES];
  };

  const char * const fixedProbeTrigger_str = "TimeStamp[378]/i:Delay[378]/s:BunchInterval[378]:ReadoutTime[378]:SequenceId[378]:BunchId[378]:Mode[378]:Source[378]";

  struct fixedProbePosition_t{
    Double_t Phi[NMR_NUM_FIXED_PROBES];
    Double_t R[NMR_NUM_FIXED_PROBES];
    Double_t Y[NMR_NUM_FIXED_PROBES];
  };

  const char * const fixedProbePosition_str = "Phi[378]/D:R[378]:Y[378]";

  struct fixedProbeHeader_t{
    UShort_t AziId[NMR_NUM_FIXED_PROBES];
    UShort_t ProbeId[NMR_NUM_FIXED_PROBES];
    UShort_t StationId[NMR_NUM_FIXED_PROBES];
    UShort_t MuxId[NMR_NUM_FIXED_PROBES];
    UShort_t RoundId[NMR_NUM_FIXED_PROBES];
    UShort_t Health[NMR_NUM_FIXED_PROBES];
    char LayerId[NMR_NUM_FIXED_PROBES];
    char YokeId[NMR_NUM_FIXED_PROBES];
    char RadId[NMR_NUM_FIXED_PROBES];
  };

  const char * const fixedProbeHeader_str = "AziId[378]/s:ProbeId[378]:StationId[378]:MuxId[378]:RoundId[378]:Health[378]:LayerId[378]/B:YokeId[378]:RadId[378]";

  //Tree FixedProbeWf
  //Branch Fid

  struct fixedProbeFid_t{
    Double_t RawFid[1548288];
    Double_t Freq[NMR_NUM_FIXED_PROBES];
    ULong64_t GpsTime[NMR_NUM_FIXED_PROBES];
    UInt_t EntryID;
  };

  const char * const fixedProbeFid_str = "RawFid[1548288]/D:Freq[378]:GpsTime[378]/l:EntryID/i";

  struct fixedProbeFullFid_t{
    Double_t RawFid[15482880];
    Double_t Freq[NMR_NUM_FIXED_PROBES];
    ULong64_t GpsTime[NMR_NUM_FIXED_PROBES];
    UInt_t EntryID;
  };

  const char * const fixedProbeFullFid_str = "RawFid[15482880]/D:Freq[378]:GpsTime[378]/l:EntryID/i";

  //Tree: trolley
  //Each entry contains 17 events (probe read outs)
  //Branch: TimeStamp
  struct trolleyTimeStamp_t{
    ULong64_t GpsCycleStart[TROLLEY_NUM_CH];
    ULong64_t InterfaceCycleStart[TROLLEY_NUM_CH];
    ULong64_t TrolleyCycleStart[TROLLEY_NUM_CH];
    ULong64_t NMRStart[TROLLEY_NUM_CH];
    ULong64_t BarcodeStart[TROLLEY_NUM_CH];
  };

  const char * const trolleyTimeStamp_str = "GpsCycleStart[17]/l:InterfaceCycleStart[17]:TrolleyCycleStart[17]:NMRStart[17]:BarcodeStart[17]";

  //Branch: ProbeFrequency
  struct trolleyProbeFrequency_t{
    Double_t Frequency[TROLLEY_NUM_CH][FREQ_METHOD_NUM];//6 methods
    Double_t FrequencyUncertainty[TROLLEY_NUM_CH][FREQ_METHOD_NUM];//6 methods
  };

  const char * const trolleyProbeFrequency_str = "Frequency[17][5]/D:FrequencyUncertainty[17][5]";

  //Branch: FieldMultipole
  struct trolleyFieldMultipole_t{
    Double_t Multipole[TROLLEY_MULTIPOLE_NUM];
    Double_t Phi;
    Double_t Time;
    Double_t Gradient;
  };

  const char * const trolleyFieldMultipole_str = "Multipole[9]/D:Phi/D:Time/D:Gradient/D";

  //Branch: ProbeSignal
  struct trolleyProbeSignal_t{
    Double_t Amplitude[TROLLEY_NUM_CH];
    Double_t FidLength[TROLLEY_NUM_CH];//before first node
    Double_t FidPower[TROLLEY_NUM_CH];
    Double_t FitChi2[TROLLEY_NUM_CH];
    Double_t FidProfile[TROLLEY_NUM_CH][PROFILE_LEN];
  };

  const char * const trolleyProbeSignal_str = "Amplitude[17]/D:FidLength[17]:FidPower[17]:FitChi2[17]:FidProfile[17][100]";

  //Branch: Position
  struct trolleyPosition_t{
    //Double_t X[TROLLEY_NUM_CH];
    //Double_t Y[TROLLEY_NUM_CH];
    Double_t Phi[TROLLEY_NUM_CH][3];//3 methods
    Double_t PosSource[TROLLEY_NUM_CH][3];//3 methods
    Double_t PosQuality[TROLLEY_NUM_CH][3];
  };

  //const char * const trolleyProbePosition_str = "X[]/D:Y[17]:Phi[17][3]:PosSource[17][3]:PosQuality[17][3]";
  const char * const trolleyProbePosition_str = "Phi[17][3]/D:PosSource[17][3]:PosQuality[17][3]";

  //Branch: Header
  struct trolleyHeader_t{
    UInt_t eventID[TROLLEY_NUM_CH];
    UShort_t serialNumber[TROLLEY_NUM_CH];
    UInt_t cycleID;
    //UShort_t health; // move to quality
  };

  const char * const trolleyHeader_str = "eventID[17]/i:serialNumber[17]/s:cycleID/i";
  //const char * const trolleyHeader_str = "eventID[17]/i:serialNumber[17]/s:health";

  //Branch: Quality
  struct trolleyQuality_t {
    Double_t FrequencyOutlier[TROLLEY_NUM_CH];   //! frequency difference of measuerement with resepecvt to interpolation (mean) from adjacent measueremens
    Double_t FrequencySmoothed[TROLLEY_NUM_CH];  //! smoothed frequency
    //Double_t Gradient;                           //! gradient calculated from dipole
    UShort_t outlier[TROLLEY_NUM_CH];            //! boolen indicating if it is an outlier
    UShort_t fidHealth[TROLLEY_NUM_CH];          //! probe wise health (bit 1: freq, bit 2: amp,  bit 3: len, bit 4: pow, bit 5: chi2, bit 6: v1)
    UInt_t   qualityFlags;                       /*! cycle health: 
						  *  bit 1: true if cycle is not complete
						  *  bits 2-18: indicates outlier of probe bit-1, same info as in outlier
						  *  bits 19-24 or of fidHealth bits
						  */
  };

  const char * const trolleyQuality_str = "FrequencyOutlier[17]/D:FrequencySmoothed[17]/D:outlier[17]/s:ProbeHealth[17]:CycleHealth/i";

  struct summaryHeader_t {
    UInt_t run;
    //ULong64_t timeStart;
    //ULong64_t timeEnd;
  };
  
  const char * const summaryHeader_str = "Run/i";//:TimeStart/l:TimeEnd/l";


  struct trolleySummary_t {
    Double_t FrequencyMean[TROLLEY_NUM_CH];
    Double_t FrequencyRMS[TROLLEY_NUM_CH];
    Double_t FrequencyMin[TROLLEY_NUM_CH];
    Double_t FrequencyMax[TROLLEY_NUM_CH];
    Double_t FidLenMean[TROLLEY_NUM_CH];
    Double_t FidLenRMS[TROLLEY_NUM_CH];
    Double_t FidAmpMean[TROLLEY_NUM_CH];
    Double_t FidAmpRMS[TROLLEY_NUM_CH];
    Double_t FidPowerMean[TROLLEY_NUM_CH];
    Double_t FidPowerRMS[TROLLEY_NUM_CH];
    Double_t ProbeHealth[TROLLEY_NUM_CH];
    Double_t ProbeResRMS[TROLLEY_NUM_CH];
    Double_t ProbeResSigma[TROLLEY_NUM_CH];
    Double_t ProbeResSym[TROLLEY_NUM_CH];
    Double_t ProbeResSymFit[TROLLEY_NUM_CH];
    Double_t VMonitor1Mean;
    Double_t VMonitor1RMS;
    Double_t VMonitor1Min;
    Double_t VMonitor1Max;
    Double_t VMonitor2Mean;
    Double_t VMonitor2RMS;
    Double_t VMonitor2Min;
    Double_t VMonitor2Max;
    Double_t TrolleyVMonitorMean;
    Double_t TrolleyVMonitorRMS;
    Double_t TrolleyVMonitorMin;
    Double_t TrolleyVMonitorMax;
    Double_t TrolleyIMonitorMean;
    Double_t TrolleyIMonitorRMS;
    Double_t TrolleyIMonitorMin;
    Double_t TrolleyIMonitorMax;
    Double_t BarcodePercentage[2];
    Double_t OverallUpPosShift[2];
    Double_t OverallPositionOffset[2];
    Double_t GroupShift;
    Double_t GroupShiftRMS;
    Int_t MajorityCodeOffset[2];
    UInt_t   SpikeNum[TROLLEY_NUM_CH];
  };

  const char * const trolleySummary_str = "FrequencyMean[17]/D:FrequencyRMS[17]:FrequencyMin[17]:FrequencyMax[17]:FidLenMean[17]:FidLenRMS[17]:FidAmpMean[17]:FidAmpRMS[17]:FidPowerMean[17]:FidPowerRMS[17]:ProbeHealth[17]:ProbeResRMS[17]:ProbeResSigma[17]:ProbeResSym[17]:ProbeResSymFit[17]:VMonitor1Mean/D:VMonitor1RMS:VMonitor1Min:VMonitor1Max:VMonitor2Mean:VMonitor2RMS:VMonitor2Min:VMonitor2Max:TrolleyVMonitorMean:TrolleyVMonitorRMS:TrolleyVMonitorMin:TrolleyVMonitorMax:TrolleyIMonitorMean:TrolleyIMonitorRMS:TrolleyIMonitorMin:TrolleyIMonitorMax:BarcodePercentage[2]:OverallUpPosShift[2]:OverallPositionOffset[2]:GroupShift:GroupShiftRMS:MajorityCodeOffset[2]/I:SpikeNum[17]/i";

  //Branch: barcode
  struct barcode_t{
    Float_t traces[17][6][300];
    Float_t adc_reference[17];
    Float_t LED_voltage[17];
    UInt_t lengths[17];
    UShort_t sampling_period[17];
    UShort_t acquisition_delay[17];
    UShort_t ref_cm[17];
  };

  const char * const barcode_str = "traces[17][6][300]/F:adc_reference[17]:LED_voltage[17]:lengths[17]/i:sampling_period[17]/s:acquisition_delay[17]:ref_cm[17]";

  //Branch: Monitor
  struct trolleyMonitor_t{
    Double_t TemperatureIn[TROLLEY_NUM_CH];
    Double_t TemperatureExt[TROLLEY_NUM_CH];
    Double_t TemperaturePSensor[TROLLEY_NUM_CH];
    Double_t InterfaceLdoTemperatureMin[TROLLEY_NUM_CH];
    Double_t InterfaceLdoTemperatureMax[TROLLEY_NUM_CH];
    Double_t Pressure[TROLLEY_NUM_CH];
    Double_t RFPowerFactor[TROLLEY_NUM_CH];
    Double_t InterfacePowerFactor[TROLLEY_NUM_CH];
    Double_t V1Min[TROLLEY_NUM_CH];
    Double_t V1Max[TROLLEY_NUM_CH];
    Double_t V2Min[TROLLEY_NUM_CH];
    Double_t V2Max[TROLLEY_NUM_CH];
    Double_t InterfaceV15negMin[TROLLEY_NUM_CH];
    Double_t InterfaceV15negMax[TROLLEY_NUM_CH];
    Double_t InterfaceV15posMin[TROLLEY_NUM_CH];
    Double_t InterfaceV15posMax[TROLLEY_NUM_CH];
    Double_t InterfaceV5Min[TROLLEY_NUM_CH];
    Double_t InterfaceV5Max[TROLLEY_NUM_CH];
    Double_t InterfaceV33Min[TROLLEY_NUM_CH];
    Double_t InterfaceV33Max[TROLLEY_NUM_CH];
    UInt_t FrameIndex[TROLLEY_NUM_CH];
    UInt_t ConfigCheckSum[TROLLEY_NUM_CH];
    UInt_t NMRCheckSum[TROLLEY_NUM_CH];
    UInt_t FrameCheckSum[TROLLEY_NUM_CH];
    UInt_t ConfigFrameSum[TROLLEY_NUM_CH];
    UInt_t NMRFrameSum[TROLLEY_NUM_CH];
    UInt_t FrameSum[TROLLEY_NUM_CH];
    UShort_t StatusBits[TROLLEY_NUM_CH];
    UShort_t Trolley_Command[TROLLEY_NUM_CH];
    UShort_t TIC_Stop[TROLLEY_NUM_CH];
    UShort_t TC_Start[TROLLEY_NUM_CH];
    UShort_t TD_Start[TROLLEY_NUM_CH];
    UShort_t TC_Stop[TROLLEY_NUM_CH];
    UShort_t Switch_RF[TROLLEY_NUM_CH];
    UShort_t PowerEnable[TROLLEY_NUM_CH];
    UShort_t RF_Enable[TROLLEY_NUM_CH];
    UShort_t Switch_Comm[TROLLEY_NUM_CH];
    UShort_t TIC_Start[TROLLEY_NUM_CH];
    UShort_t Cycle_Length[TROLLEY_NUM_CH];
    UShort_t Power_Control_1[TROLLEY_NUM_CH];
    UShort_t Power_Control_2[TROLLEY_NUM_CH];
    UShort_t InterfaceRFSwitchOffset[TROLLEY_NUM_CH];
    UShort_t InterfaceCommSwitchOffset[TROLLEY_NUM_CH];
    UShort_t InterfacePowerProtectionTrip[TROLLEY_NUM_CH];
    UShort_t InterfacePowerStatus[TROLLEY_NUM_CH];
  };

  const char * const trolleyMonitor_str = "TemperatureIn[17]/D:TemperatureExt[17]:TemperaturePSensor[17]:InterfaceLdoTemperatureMin[17]:InterfaceLdoTemperatureMax[17]:Pressure[17]:RFPowerFactor[17]:InterfacePowerFactor[17]:V1Min[17]:V1Max[17]:V2Min[17]:V2Max[17]:InterfaceV15negMin[17]:InterfaceV15negMax[17]:InterfaceV15posMin[17]:InterfaceV15posMax[17]:InterfaceV5Min[17]:InterfaceV5Max[17]:InterfaceV33Min[17]:InterfaceV33Max[17]:FrameIndex[17]/i:ConfigCheckSum[17]:NMRCheckSum[17]:FrameCheckSum[17]:ConfigFrameSum[17]:NMRFrameSum[17]:FrameSum[17]:StatusBits/s:Trolley_Command[17]:TIC_Stop[17]:TC_Start[17]:TD_Start[17]:TC_Stop[17]:Switch_RF[17]:PowerEnable[17]:RF_Enable[17]:Switch_Comm[17]:TIC_Start[17]:Cycle_Length[17]:Power_Control_1[17]:Power_Control_2[17]:InterfaceRFSwitchOffset[17]:InterfaceCommSwitchOffset[17]:InterfacePowerProtectionTrip[17]:InterfacePowerStatus[17]";

  //Tree Analyzed Regular Barcode
  struct regBarcode_t{
    Double_t Position;
    Double_t PositionOffset;
    Int_t AbsCode;
    Int_t AbsIdx;
    Int_t AbsShift;
    Int_t Status;
  };

  const char * const regBarcode_str = "Position/D:PositionOffset:AbsCode/I:AbsIdx:AbsShift:Status";

  //Tree Analyzed Absolute Barcode
  struct absBarcode_t{
     Double_t UpPos;
     Double_t UpPosMatched;
     Double_t GlobalPos;
     Double_t PosOffset;
     Int_t Code;
     Int_t CodeMatched;
  };

  const char * const absBarcode_str = "UpPos/D:UpPosMatched:GlobalPos:PosOffset:Code/I:CodeMatched";

  //Tree trolleyWf
  //Branch Fid

  struct trolleyFid_t{
    Double_t RawFid[272000];
    Double_t Freq[TROLLEY_NUM_CH];
    ULong64_t GpsTime[TROLLEY_NUM_CH];
    UInt_t EntryID;
  };

  const char * const trolleyFid_str = "RawFid[272000]/D:Freq[17]:GpsTime[17]/l:EntryID/i";

  //Tree: galil
  //Branch: Trolley
  struct galilTrolley_t{
    Double_t TimeStamp;
    ULong64_t SysTimeStamp;
    Double_t Tensions[2];
    Double_t Temperatures[2];
    Double_t ControlVoltages[3];
    Double_t Positions[3];
    Double_t Velocities[3];
  };

  const char * const galilTrolley_str = "TimeStamp/D:SysTimeStamp/l:Tensions[2]/D:Temperatures[2]:ControlVoltages[3]:Positions[3]:Velocities[3]";

  //Branch: Galil Trolley Summary
  struct galilTrolleySummary_t {
      Double_t tempFishMin;
      Double_t tempFishMax;
      Double_t tempFishMean;
      Double_t tempFishRMS;
      Double_t tempCoaxMin;
      Double_t tempCoaxMax;
      Double_t tempCoaxMean;
      Double_t tempCoaxRMS;
      Double_t tensionFishMin;
      Double_t tensionFishMax;
      Double_t tensionFishMean;
      Double_t tensionFishRMS;
      Double_t tensionCoaxMin;
      Double_t tensionCoaxMax;
      Double_t tensionCoaxMean;
      Double_t tensionCoaxRMS;
      Double_t ActiveTimeFish;
      Double_t ActiveTimeCoax;
      Double_t ActiveTimeGarage;
      UInt_t   healthFish;
      UInt_t   healthCoax;
  };

  const char * const galilTrolleySummary_str = "tempFishMin/D:tempFishMax:tempFishMean:tempFishRMS:tempCoaxMin:tempCoaxMax:tempCoaxMean:tempCoaxRMS:tensionFishMin:tensionFishMax:tensionFishMean:tensionFishRMS:tensionCoaxMin:tensionCoaxMax:tensionCoaxMean:tensionCoaxRMS:ActiveTimeFish/D:ActiveTimeCoax/D:ActiveTimeGarage/D:healthFish/i:healthCoax/i";

  //Branch Fixed Probe Summary
  struct fixedProbeSummary_t {
    Double_t freqMean[NMR_NUM_FIXED_PROBES];
    Double_t freqRMS[NMR_NUM_FIXED_PROBES];
    Double_t freqMax[NMR_NUM_FIXED_PROBES];
    Double_t freqMin[NMR_NUM_FIXED_PROBES];
    Double_t freqDrift[NMR_NUM_FIXED_PROBES];
    Double_t freqErrMean[NMR_NUM_FIXED_PROBES];
    Double_t freqErrRMS[NMR_NUM_FIXED_PROBES];
    Double_t fidNoiseMean[NMR_NUM_FIXED_PROBES];
    Double_t fidNoiseRMS[NMR_NUM_FIXED_PROBES];
    Double_t fidAmpMean[NMR_NUM_FIXED_PROBES];
    Double_t fidAmpRMS[NMR_NUM_FIXED_PROBES];
    Double_t fidLenMean[NMR_NUM_FIXED_PROBES];
    Double_t fidLenRMS[NMR_NUM_FIXED_PROBES];
    Double_t fidPowerMean[NMR_NUM_FIXED_PROBES];
    Double_t fidPowerRMS[NMR_NUM_FIXED_PROBES];
    //UInt_t   fidHealth[NMR_NUM_FIXED_PROBES];
//    UInt_t   missedFid[NMR_NUM_FIXED_PROBES];
    Double_t probeResMean[NMR_NUM_FIXED_PROBES];
    Double_t probeResSigma[NMR_NUM_FIXED_PROBES];
    Int_t probeHealth[NMR_NUM_FIXED_PROBES];
  };

  const char * const fixedProbeSummary_str = "freqMean[378]/D:freqRMS[378]:freqMax[378]:freqMin[378]:freqDrift[378]:freqErrMean[378]:freqErrRMS[378]:fidNoiseMean[378]:fidNoiseRMS[378]:fidAmpMean[378]:fidAmpRMS[378]:fidLenMean[378]:fidLenRMS[378]:fidPowerMean[378]:fidPowerRMS[378]:probeResMean[378]/D:probeResSigma[378]/D:probeHealth[378]/I";

  //Branch Fluxgate Summary
  struct fluxgateSummary_t {
    Double_t rate;
    Double_t probeId[FLUXGATE_NUM_CH];
    Double_t orientation[FLUXGATE_NUM_CH];
    Double_t mode[FLUXGATE_NUM_CH];
    Double_t ampMin[FLUXGATE_NUM_CH];
    Double_t ampMax[FLUXGATE_NUM_CH];
    Double_t ampMean[FLUXGATE_NUM_CH];
    Double_t ampRMS[FLUXGATE_NUM_CH];
    UInt_t channelHealth[FLUXGATE_NUM_CH];
    UInt_t probeHealth[FLUXGATE_NUM_CH/6];
  };

  const char * const fluxgateSummary_str = "rate/D:probeId[24]/D:orientation[24]/D:mode[24]/D:ampMin[24]/D:ampMax[24]/D:ampMean[24]/D:ampRMS[24]/D:channelHealth[24]/i:probeHealth[4]/i";

  //Branch PSFeedback Summary
  struct psFeedbackSummary_t {
    SUM_VAR(Double_t, current)
    SUM_VAR(Double_t, voltage)
    SUM_VAR(Double_t, filtered_mean_freq)
    Int_t all_enabled;
    Int_t fdbk_state_changed;
    Int_t field_setpoint_changed;
    Int_t P_coeff_changed;
    Int_t I_coeff_changed;
    Int_t I_alt_coeff_changed;
    Int_t D_coeff_changed;
  };

  const char * const psFeedbackSummary_str = "currentMin/D:currentMax/D:currentMean/D:currentRMS/D:voltageMin/D:voltageMax/D:voltageMean/D:voltageRMS/D:filteredMeanfreqMin/D:filteredMeanfreqMax/D:filteredMeanfreqMean/D:filteredMeanfreqRMS/D:all_enabled/I:fdbk_state_changed:P_coeff_changed:I_coeff_changed:I_alt_coeff_changed:D_coeff_changed";

  //Branch Surfacecoil Summary
  struct surfaceCoilSummary_t { 
    SUM_VAR_ARRAY(Double_t, topCurrent, SC_NUM_COILS)
    SUM_VAR_ARRAY(Double_t, botCurrent, SC_NUM_COILS)
    SUM_VAR_ARRAY(Double_t, azCurrent,  SC_NUM_AZ_COILS)
    SUM_VAR_ARRAY(Double_t, topTemp,    SC_NUM_COILS)
    SUM_VAR_ARRAY(Double_t, botTemp,    SC_NUM_COILS)
    SUM_VAR_ARRAY(Double_t, azTemp,     SC_NUM_AZ_COILS)
    Int_t topHealth[SC_NUM_COILS]; 
    Int_t botHealth[SC_NUM_COILS];
    Int_t azHealth[SC_NUM_AZ_COILS];
  };

  const char * const surfaceCoilSummary_str = "topCurrentMin[100]/D:topCurrentMax[100]/D:topCurrentMean[100]/D:topCurrentRMS[100]/D:botCurrentMin[100]/D:botCurrentMax[100]/D:botCurrentMean[100]/D:botCurrentRMS[100]/D:azCurrentMin[4]/D:azCurrentMax[4]/D:azCurrentMean[4]/D:azCurrentRMS[100]/D:topTempMin[100]/D:topTempMax[100]/D:topTempMean[100]/D:topTempRMS[100]/D:botTempMin[100]/D:botTempMax[100]/D:botTempMean[100]/D:botTempRMS[100]/D:azTempMin[4]/D:azTempMax[4]/D:azTempMean[4]/D:azTempRMS/D:topHealth[100]/I:botHealth[100]/I:azHealth[4]/I";

  struct fixedProbeQuality_t {
    Int_t qtag[NMR_NUM_FIXED_PROBES];
  };

  const char * const fixedProbeQuality_str = "qtag[378]/I";

  //Branch: Plunging Probe
  struct galilPlungingProbe_t{
    Double_t TimeStamp;
    ULong64_t SysTimeStamp;
    Double_t Positions[3];
    Double_t Velocities[3];
    Double_t OutputVs[3];
  };

  const char * const galilPlungingProbe_str = "TimeStamp/D:SysTimeStamp/L:Positions[3]/D:Velocities[3]:OutputVs[3]";

  //Tree: surface coils
  struct surfaceCoils_t{
    ULong64_t BotTime[SC_NUM_COILS];
    ULong64_t TopTime[SC_NUM_COILS];
    Double_t BotCurrentSetPoints[SC_NUM_COILS];
    Double_t TopCurrentSetPoints[SC_NUM_COILS];
    Double_t BotCurrents[SC_NUM_COILS];
    Double_t TopCurrents[SC_NUM_COILS];
    Double_t BotTemps[SC_NUM_COILS];
    Double_t TopTemps[SC_NUM_COILS];
    Double_t AzCurrentSetPoints[SC_NUM_AZ_COILS];
    Double_t AzCurrents[SC_NUM_AZ_COILS];
    Double_t AzTemps[SC_NUM_AZ_COILS];
    Int_t QTag;
  };

  const char * const surfaceCoils_str = "BotTime[100]/l:TopTime[100]/l:BotCurrentSetPoints[100]/D:TopCurrentSetPoints[100]/D:BotCurrents[100]/D:TopCurrents[100]/D:BotTemps[100]/D:TopTemps[100]/D:AzCurrentSetPoints[4]/D:AzCurrents[4]/D:AzTemps[4]/D:QTag/I";
  
  //Tree: ps-feedback
  struct psFeedback_t{
    unsigned long int sys_clock;   // system clock (UTC time integer in ns)  
    unsigned long int gps_clock;   // gps clock (UTC time integer in ns)  
    double current;                // current in amps 
    double voltage;                // voltage in amps  
    double field_setpoint;         // field setpoint (kHz); used when feedback is active      
    double current_setpoint;       // current setpoint (amps); used when feedback is inactive   
    double filtered_mean_freq;     // filtered mean frequency (kHz); from fixed probe DAQ         
    double P_coeff;                // P coefficient (PID loop)     
    double I_coeff;                // I coefficient (PID loop) 
    double I_alt_coeff;            // I alt coefficient (PID loop) 
    double D_coeff;                // D coefficient (PID loop) 
    //std::string mode;              // yokogawa mode; VOLT or CURR; UDEF = undefined 
    Int_t is_enabled;               // output enable state 
    Int_t fdbk_state;               // fine,coarse or hold current
    Int_t QTag;				//Quality tag
    Short_t ProbeList[378];       //List of feedback probes
  };

  const char * const psFeedback_str = "SysClock/l:GpsClock/l:Current/D:Voltage/D:FieldSetPoint/D:CurrentSetPoint/D:FilteredMeanFreq/D:PCoeff/D:ICoeff/D:IAltCoeff/D:DCoeff/D:is_enabled/I:fdbk_stat/I:QTag/I:ProbeList[378]/S";

  //Tree: fluxgate
  struct fluxgate_wf_t{
    unsigned long int sys_clock;
    double r[4];
    double theta[4];
    double z[4];
    double eff_rate;
    double nFluxgate;
    double len;
    double time_interval;
    double trace[24][15];
    int adc_fg_id[24]; // labels which probe is plugged into which adc channel
    int adc_fg_ch[24]; // labels which axis of probe is plugged into which adc channel
    int adc_fg_dcac[24]; // labels whether adc channel is ac coupled
  };

  const char * const fluxgate_wf_str = "sys_clock/l:fg_r[4]/D:fg_theta[4]/D:fg_z[4]/D:eff_rate/D:nFluxgate/D:len/D:time_interval/D:trace[24][15]/D:adc_fg_id[24]/I:adc_fg_ch[24]/I:adc_fg_dcac[24]/I";

  struct fluxgate_fft_t{
    double len;
    double freq_interval;
    double fft_trace[24][260];
  };

  const char * const fluxgate_fft_str = "len/D:freq_interval/D:fft_trace[24][260]/D";

  //Tree: PlungingProbe

  //Branch: Info
  struct plungingProbeInfo_t{
    ULong64_t TimeStamp;
    Double_t Temperature;
    Double_t R;
    Double_t Y;
    Double_t Phi;
    Double_t Amplitude;
    Double_t FidLength;
    Double_t FidPower;
    Double_t FitChi2;
    Double_t FidProfile[PROFILE_LEN];
    UShort_t FlayRunNumber;
    UShort_t ProbeIndex;
  };

  const char * const plungingProbeInfo_str= "TimeStamp/l:Temperature/D:R:Y:Phi:Amplitude:FidLength:FidPower:FitChi2:FidProfile[100]:FlayRunNumber/s:ProbeIndex";

  //Branch: Frequency
  struct plungingProbeFrequency_t{
    Double_t F0;
    Double_t FLO;
    Double_t Frequency[FREQ_METHOD_NUM];
    Double_t FrequencyUncertainty[FREQ_METHOD_NUM];
  };

  const char * const plungingProbeFrequency_str= "F0/D:FLO:Frequency[5]:FrequencyUncertainty[5]";

  //Tree: AbsoluteProbe

  //Branch: Info
  struct absoluteProbeInfo_t{
    ULong64_t TimeStamp;
    Double_t Temperature;
    Double_t X;
    Double_t Y;
    Double_t Z;
    Double_t Amplitude;
    Double_t FidLength;
    Double_t FidPower;
    Double_t FitChi2;
    Double_t FidProfile[PROFILE_LEN];
    UShort_t FlayRunNumber;
    UShort_t ProbeIndex;
  };

  const char * const absoluteProbeInfo_str= "TimeStamp/l:Temperature/D:X:Y:Z:Amplitude:FidLength:FidPower:FitChi2:FidProfile[100]:FlayRunNumber/s:ProbeIndex";

  //Branch: Frequency
  struct absoluteProbeFrequency_t{
    Double_t F0;
    Double_t FLO;
    Double_t Frequency[FREQ_METHOD_NUM];
    Double_t FrequencyUncertainty[FREQ_METHOD_NUM];
  };

  const char * const absoluteProbeFrequency_str= "F0/D:FLO:Frequency[5]:FrequencyUncertainty[5]";

  //Generic user data branch
  struct user_data_t {
    double values[8];
    char comments[64];
  };

  const char * const user_data_str = "values[8]/D:comments[64]/C";

  //Branch Issues
  struct issue_t {
    ULong64_t timestamp;
    Double_t values[400][8];
    UInt_t ids[400];
    UInt_t run;
    UInt_t type;
    UInt_t system;
    UInt_t event;
    UInt_t n_id;
    char comments[128];
  };

  const char * const issue_str = "timestamp/l:values[400][8]/D:ids[400]/i:run/i:type/i:system/i:event/i:n_id/i:comments[128]/C";

} // ::g2field

#undef SUM_VAR
#undef SUM_VAR_ARRAY
#undef SUM_STR

#endif
