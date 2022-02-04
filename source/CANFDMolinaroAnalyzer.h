#ifndef CANFDMOLINARO_ANALYZER_H
#define CANFDMOLINARO_ANALYZER_H

//--------------------------------------------------------------------------------------------------

#include <Analyzer.h>
#include "CANFDMolinaroAnalyzerResults.h"
#include "CANFDMolinaroSimulationDataGenerator.h"

//--------------------------------------------------------------------------------------------------

class CANFDMolinaroAnalyzerSettings;

//--------------------------------------------------------------------------------------------------


class ANALYZER_EXPORT CANFDMolinaroAnalyzer : public Analyzer2 {

  public: CANFDMolinaroAnalyzer();

  public: virtual ~CANFDMolinaroAnalyzer();

  public: virtual void SetupResults();

  public: virtual void WorkerThread();

  public: virtual U32 GenerateSimulationData (U64 newest_sample_requested,
                                              U32 sample_rate,
                                              SimulationChannelDescriptor** simulation_channels);
  public: virtual U32 GetMinimumSampleRateHz () ;

  public: virtual const char* GetAnalyzerName() const ;

  public: virtual bool NeedsRerun () ;

//--- Protected properties
  protected: std::auto_ptr< CANFDMolinaroAnalyzerSettings > mSettings;
  protected: std::auto_ptr< CANFDMolinaroAnalyzerResults > mResults;
  protected: AnalyzerChannelData* mSerial;

  protected: CANMolinaroSimulationDataGenerator mSimulationDataGenerator;
   protected: bool mSimulationInitilized;

  protected: U32 mSampleRateHz;


//---------------- CAN decoder
  private: U64 mStartOfFieldSampleNumber ;
  private: U64 mStartOfFrameSampleNumber ;
  private: U32 mCurrentSamplesPerBit ;

//--- CAN protocol
  private: typedef enum  {
    IDLE, IDENTIFIER, CONTROL_BASE, CONTROL_EXTENDED, CONTROL_AFTER_R0, DATA, SBC,
    CRC15, CRC17, CRC21, CRCDEL, ACK, ENDOFFRAME, INTERMISSION, DECODER_ERROR
  } FrameFieldEngineState ;

  private: FrameFieldEngineState mFrameFieldEngineState ;
  private: int mFieldBitIndex ;
  private: int mConsecutiveBitCountOfSamePolarity ;
  private: bool mPreviousBit ;
  private: bool mUnstuffingActive ;

//--- Received frame
  private: uint32_t mIdentifier ;
  private: U32 mSBCField ;
  private: U32 mStuffBitCount ;
  private: U32 mDataCodeLength ;
  private: U8 mData [64] ;
  private: U16 mCRC15Accumulator ;
  private: U16 mCRC15 ;
  private: U32 mCRC17Accumulator ;
  private: U32 mCRC17 ;
  private: U32 mCRC21Accumulator ;
  private: U32 mCRC21 ;
  private: typedef enum {base, extended} FrameFormat ;
  private: FrameFormat mFrameFormat ;
  private: typedef enum {canData, remote, canfdData} FrameType ;
  private: FrameType mFrameType ;
  private: bool mBRS ;
  private: bool mESI ;

//---------------- CAN decoder methods
  private: void enterBit (const bool inBit, const U64 inSampleNumber) ;
  private: void decodeFrameBit (const bool inBit, const U64 inSampleNumber) ;
  private: void enterBitInCRC15 (const bool inBit) ;
  private: void enterBitInCRC17 (const bool inBit) ;
  private: void enterBitInCRC21 (const bool inBit) ;
  private: void addMark (const U64 inSampleNumber, const AnalyzerResults::MarkerType inMarker) ;
  private: void addBubble (const U8 inBubbleType,
                           const U64 inData1,
                           const U64 inData2,
                           const U64 inEndSampleNumber) ;
  private: void enterInErrorMode (const U64 inSampleNumber) ;

  private: void handle_IDLE_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_IDENTIFIER_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_CONTROL_BASE_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_CONTROL_EXTENDED_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_CONTROL_AFTER_R0_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_DATA_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_SBC_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_CRC15_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_CRC17_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_CRC21_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_CRCDEL_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_ACK_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_ENDOFFRAME_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_INTERMISSION_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_DECODER_ERROR_state (const bool inBit, const U64 inSampleNumber) ;
} ;

//--------------------------------------------------------------------------------------------------

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer( );
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

//--------------------------------------------------------------------------------------------------

#endif //CANFDMOLINARO_ANALYZER_H
