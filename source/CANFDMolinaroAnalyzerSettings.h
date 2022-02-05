#ifndef CANMOLINARO_ANALYZER_SETTINGS
#define CANMOLINARO_ANALYZER_SETTINGS

//--------------------------------------------------------------------------------------------------

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

//--------------------------------------------------------------------------------------------------

typedef enum {
  CANFD_ISO_PROTOCOL,
  CANFD_NON_ISO_PROTOCOL
} ProtocolSetting ;

//--------------------------------------------------------------------------------------------------

typedef enum {
  GENERATE_BIT_DOMINANT,
  GENERATE_BIT_RECESSIVE,
  GENERATE_BIT_RANDOMLY
} SimulatorGeneratedBit ;

//--------------------------------------------------------------------------------------------------

typedef enum {
  GENERATE_ALL_FRAME_TYPES,
  GENERATE_ONLY_STANDARD_DATA,
  GENERATE_ONLY_EXTENDED_DATA,
  GENERATE_ONLY_STANDARD_REMOTE,
  GENERATE_ONLY_EXTENDED_REMOTE,
  GENERATE_ONLY_CANFD_BASE_0_16,
  GENERATE_ONLY_CANFD_EXTENDED_0_16,
  GENERATE_ONLY_CANFD_BASE_20_64,
  GENERATE_ONLY_CANFD_EXTENDED_20_64
} SimulatorGeneratedFrameType ;

//--------------------------------------------------------------------------------------------------

class CANFDMolinaroAnalyzerSettings : public AnalyzerSettings {
public:
  CANFDMolinaroAnalyzerSettings();
  virtual ~CANFDMolinaroAnalyzerSettings();

  virtual bool SetSettingsFromInterfaces();
  void UpdateInterfacesFromSettings();
  virtual void LoadSettings( const char* settings );
  virtual const char* SaveSettings();


  Channel mInputChannel;

  public: U32 arbitrationBitRate (void) const { return mArbitrationBitRate ; }

  public: U32 dataBitRate (void) const { return mDataBitRate ; }

  public: bool inverted (void) const { return mInverted ; }

  public: SimulatorGeneratedBit generatedAckSlot (void) const {
    return mSimulatorGeneratedAckSlot ;
  }

  public: SimulatorGeneratedBit generatedBSRSlot (void) const {
    return mSimulatorGeneratedBSRSlot ;
  }

  public: SimulatorGeneratedBit generatedESISlot (void) const {
    return mSimulatorGeneratedESISlot ;
  }

  public: SimulatorGeneratedFrameType generatedFrameType (void) const {
   return mSimulatorGeneratedFrameType ;
  }

  public: ProtocolSetting protocol (void) const {
   return mProtocol ;
  }

  public: U32 simulatorRandomSeed (void) const {
   return mSimulatorRandomSeed ;
  }

  public: U32 arbitrationSegment2 (void) const {
   return mArbitrationSegment2 ;
  }

  public: U32 dataSegment2 (void) const {
   return mDataSegment2 ;
  }

  protected: std::auto_ptr <AnalyzerSettingInterfaceChannel> mInputChannelInterface ;
  protected: std::auto_ptr <AnalyzerSettingInterfaceInteger> mArbitrationBitRateInterface ;
  protected: std::auto_ptr <AnalyzerSettingInterfaceInteger> mDataBitRateInterface ;
  protected: std::auto_ptr <AnalyzerSettingInterfaceInteger> mArbitrationSegment2Interface ;
  protected: std::auto_ptr <AnalyzerSettingInterfaceInteger> mDataSegment2Interface ;
  protected: std::auto_ptr <AnalyzerSettingInterfaceNumberList> mCanChannelInvertedInterface ;
  protected: std::auto_ptr <AnalyzerSettingInterfaceNumberList> mSimulatorAckGenerationInterface ;
  protected: std::auto_ptr <AnalyzerSettingInterfaceNumberList> mSimulatorESIGenerationInterface ;
  protected: std::auto_ptr <AnalyzerSettingInterfaceNumberList> mSimulatorBSRGenerationInterface ;
  protected: std::auto_ptr <AnalyzerSettingInterfaceNumberList> mSimulatorFrameTypeGenerationInterface ;
  protected: std::auto_ptr <AnalyzerSettingInterfaceNumberList> mProtocolInterface ;
  protected: std::auto_ptr <AnalyzerSettingInterfaceInteger> mSimulatorRandomSeedInterface ;

  protected: U32 mArbitrationBitRate ;
  protected: U32 mDataBitRate ;
  protected: U32 mSimulatorRandomSeed ;
  protected: U32 mArbitrationSegment2 = 25 ;
  protected: U32 mDataSegment2 = 25 ;
  protected: SimulatorGeneratedBit mSimulatorGeneratedAckSlot = GENERATE_BIT_DOMINANT ;
  protected: SimulatorGeneratedBit mSimulatorGeneratedESISlot = GENERATE_BIT_DOMINANT ;
  protected: SimulatorGeneratedBit mSimulatorGeneratedBSRSlot = GENERATE_BIT_DOMINANT ;
  protected: SimulatorGeneratedFrameType mSimulatorGeneratedFrameType = GENERATE_ALL_FRAME_TYPES ;
  protected: ProtocolSetting mProtocol = CANFD_ISO_PROTOCOL ;
  protected: bool mInverted = false ;
};

//--------------------------------------------------------------------------------------------------

#endif //CANMOLINARO_ANALYZER_SETTINGS
