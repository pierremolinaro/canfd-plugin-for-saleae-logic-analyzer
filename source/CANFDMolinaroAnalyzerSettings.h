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

protected:
  std::auto_ptr< AnalyzerSettingInterfaceChannel >  mInputChannelInterface;
  std::auto_ptr< AnalyzerSettingInterfaceInteger >  mArbitrationBitRateInterface;
  std::auto_ptr< AnalyzerSettingInterfaceInteger >  mDataBitRateInterface;
  std::auto_ptr< AnalyzerSettingInterfaceNumberList > mCanChannelInvertedInterface ;
  std::auto_ptr< AnalyzerSettingInterfaceNumberList > mSimulatorAckGenerationInterface ;
  std::auto_ptr< AnalyzerSettingInterfaceNumberList > mSimulatorESIGenerationInterface ;
  std::auto_ptr< AnalyzerSettingInterfaceNumberList > mSimulatorBSRGenerationInterface ;
  std::auto_ptr< AnalyzerSettingInterfaceNumberList > mSimulatorFrameTypeGenerationInterface ;
  std::auto_ptr< AnalyzerSettingInterfaceNumberList > mProtocolInterface ;

  U32 mArbitrationBitRate ;
  U32 mDataBitRate ;
  SimulatorGeneratedBit mSimulatorGeneratedAckSlot = GENERATE_BIT_DOMINANT ;
  SimulatorGeneratedBit mSimulatorGeneratedESISlot = GENERATE_BIT_DOMINANT ;
  SimulatorGeneratedBit mSimulatorGeneratedBSRSlot = GENERATE_BIT_DOMINANT ;
  SimulatorGeneratedFrameType mSimulatorGeneratedFrameType = GENERATE_ALL_FRAME_TYPES ;
  ProtocolSetting mProtocol = CANFD_ISO_PROTOCOL ;
  bool mInverted = false ;
};

//--------------------------------------------------------------------------------------------------

#endif //CANMOLINARO_ANALYZER_SETTINGS
