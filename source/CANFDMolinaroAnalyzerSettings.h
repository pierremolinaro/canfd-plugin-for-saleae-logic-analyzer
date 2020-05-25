#ifndef CANMOLINARO_ANALYZER_SETTINGS
#define CANMOLINARO_ANALYZER_SETTINGS

//--------------------------------------------------------------------------------------------------

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

//--------------------------------------------------------------------------------------------------

typedef enum {
  GENERATE_ACK_DOMINANT,
  GENERATE_ACK_RECESSIVE,
  GENERATE_ACK_RANDOMLY
} SimulatorGeneratedAckSlot ;

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
  U32 mBitRate;

  public: bool inverted (void) const { return mInverted ; }

  public: SimulatorGeneratedAckSlot generatedAckSlot (void) const {
    return mSimulatorGeneratedAckSlot ;
  }

  public: SimulatorGeneratedFrameType generatedFrameType (void) const {
   return mSimulatorGeneratedFrameType ;
  }

protected:
  std::auto_ptr< AnalyzerSettingInterfaceChannel >  mInputChannelInterface;
  std::auto_ptr< AnalyzerSettingInterfaceInteger >  mBitRateInterface;
  std::auto_ptr< AnalyzerSettingInterfaceNumberList > mCanChannelInvertedInterface ;
  std::auto_ptr< AnalyzerSettingInterfaceNumberList > mSimulatorAckGenerationInterface ;
  std::auto_ptr< AnalyzerSettingInterfaceNumberList > mSimulatorFrameTypeGenerationInterface ;

  SimulatorGeneratedAckSlot mSimulatorGeneratedAckSlot = GENERATE_ACK_DOMINANT ;
  SimulatorGeneratedFrameType mSimulatorGeneratedFrameType = GENERATE_ALL_FRAME_TYPES ;
  bool mInverted = false ;
};

//--------------------------------------------------------------------------------------------------

#endif //CANMOLINARO_ANALYZER_SETTINGS
