#include "CANFDMolinaroAnalyzerSettings.h"
#include <AnalyzerHelpers.h>

//--------------------------------------------------------------------------------------------------

CANFDMolinaroAnalyzerSettings::CANFDMolinaroAnalyzerSettings() :
mInputChannel (UNDEFINED_CHANNEL),
mArbitrationBitRate (125 * 1000) {
	mInputChannelInterface.reset (new AnalyzerSettingInterfaceChannel ());
	mInputChannelInterface->SetTitleAndTooltip ("Serial", "Standard Molinaro's CAN");
	mInputChannelInterface->SetChannel (mInputChannel);

	mBitRateInterface.reset (new AnalyzerSettingInterfaceInteger ()) ;
	mBitRateInterface->SetTitleAndTooltip ("CAN Artitration Bit Rate (bit/s)",
                                         "Specify the CAN arbitration bit rate in bits per second." );
	mBitRateInterface->SetMax (25 * 1000 * 1000) ;
	mBitRateInterface->SetMin (1) ;
	mBitRateInterface->SetInteger (mArbitrationBitRate) ;

//--- Add Channel level inversion
	mCanChannelInvertedInterface.reset (new AnalyzerSettingInterfaceNumberList ( )) ;
	mCanChannelInvertedInterface->SetTitleAndTooltip ("Dominant Logic Level", "" );
  mCanChannelInvertedInterface->AddNumber (0.0,
                                           "Low",
                                           "Low is the usual dominant level") ;
  mCanChannelInvertedInterface->AddNumber (1.0,
                                           "High",
                                           "High is the inverted dominant level") ;

//--- Add Protocol
	mProtocolInterface.reset (new AnalyzerSettingInterfaceNumberList ( )) ;
	mProtocolInterface->SetTitleAndTooltip ("CANFD Protocol", "" );
  mProtocolInterface->AddNumber (0.0, "ISO", "") ;
  mProtocolInterface->AddNumber (1.0, "Non IS0", "") ;

//--- Simulator ACK level
  mSimulatorAckGenerationInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
	mSimulatorAckGenerationInterface->SetTitleAndTooltip ("Simulator ACK SLOT generated level", "");
  mSimulatorAckGenerationInterface->AddNumber (0.0,
                                               "Dominant",
                                               "Dominant is the valid level for ACK SLOT") ;
  mSimulatorAckGenerationInterface->AddNumber (1.0,
                                               "Recessive",
                                               "Recessive is the invalid level for ACK SLOT") ;
  mSimulatorAckGenerationInterface->AddNumber (2.0,
                                               "Random",
                               "The simulator generates dominant or recessive level randomly") ;

//--- Simulator ESI level
  mSimulatorESIGenerationInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
	mSimulatorESIGenerationInterface->SetTitleAndTooltip ("Simulator ESI generated level", "");
  mSimulatorESIGenerationInterface->AddNumber (0.0,
                                               "Dominant",
                                               "Dominant means the sender is error active") ;
  mSimulatorESIGenerationInterface->AddNumber (1.0,
                                               "Recessive",
                                               "Recessive means the sender is error passive") ;
  mSimulatorESIGenerationInterface->AddNumber (2.0,
                                               "Random",
                               "The simulator generates dominant or recessive level randomly") ;

//--- Simulator Generated frames
  mSimulatorFrameTypeGenerationInterface.reset (new AnalyzerSettingInterfaceNumberList ()) ;
	mSimulatorFrameTypeGenerationInterface->SetTitleAndTooltip ("Simulator Generated Frames", "");
  mSimulatorFrameTypeGenerationInterface->AddNumber (0.0, "All Types", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (1.0, "Only CAN2.0B Standard Data Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (2.0, "Only CAN2.0B Extended Data Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (3.0, "Only CAN2.0B Standard Remote Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (4.0, "Only CAN2.0B Extended Remote Frames", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (5.0, "Only CANFD Base Data Frames, 0-16 bytes", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (6.0, "Only CANFD Extended Data Frames, 0-16 bytes", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (7.0, "Only CANFD Base Data Frames, 20-64 bytes", "") ;
  mSimulatorFrameTypeGenerationInterface->AddNumber (8.0, "Only CANFD Extended Data Frames, 20-64 bytes", "") ;
  mSimulatorFrameTypeGenerationInterface->SetNumber (0.0) ;

//--- Install interfaces
	AddInterface (mInputChannelInterface.get ()) ;
	AddInterface (mBitRateInterface.get ());
	AddInterface (mCanChannelInvertedInterface.get ());
	AddInterface (mProtocolInterface.get ());
	AddInterface (mSimulatorAckGenerationInterface.get ());
	AddInterface (mSimulatorFrameTypeGenerationInterface.get ());
	AddInterface (mSimulatorESIGenerationInterface.get ());

	AddExportOption( 0, "Export as text/csv file" );
	AddExportExtension( 0, "text", "txt" );
	AddExportExtension( 0, "csv", "csv" );

	ClearChannels ();
	AddChannel (mInputChannel, "Serial", false) ;
}

//--------------------------------------------------------------------------------------------------

CANFDMolinaroAnalyzerSettings::~CANFDMolinaroAnalyzerSettings(){
}

//--------------------------------------------------------------------------------------------------

bool CANFDMolinaroAnalyzerSettings::SetSettingsFromInterfaces () {
	mInputChannel = mInputChannelInterface->GetChannel();

	mArbitrationBitRate = mBitRateInterface->GetInteger();

  mInverted = U32 (mCanChannelInvertedInterface->GetNumber ()) != 0 ;

  mProtocol = ProtocolSetting (mProtocolInterface->GetNumber ()) ;

  mSimulatorGeneratedAckSlot
    = SimulatorGeneratedBit (mSimulatorAckGenerationInterface->GetNumber ()) ;

  mSimulatorGeneratedFrameType
    = SimulatorGeneratedFrameType (mSimulatorFrameTypeGenerationInterface->GetNumber ()) ;

  mSimulatorGeneratedESISlot
    = SimulatorGeneratedBit (mSimulatorESIGenerationInterface->GetNumber ()) ;

	ClearChannels();
	AddChannel (mInputChannel, "CANFD", true) ;

	return true;
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerSettings::UpdateInterfacesFromSettings () {
	mInputChannelInterface->SetChannel (mInputChannel) ;
	mBitRateInterface->SetInteger (mArbitrationBitRate) ;
	mCanChannelInvertedInterface->SetNumber (double (mInverted)) ;
	mProtocolInterface->SetNumber (double (mProtocol)) ;
  mSimulatorAckGenerationInterface->SetNumber (mSimulatorGeneratedAckSlot) ;
  mSimulatorFrameTypeGenerationInterface->SetNumber (mSimulatorGeneratedFrameType) ;
  mSimulatorESIGenerationInterface->SetNumber (mSimulatorGeneratedESISlot) ;
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerSettings::LoadSettings (const char* settings) {
  U32 value ;
	SimpleArchive text_archive;
	text_archive.SetString (settings) ;

	text_archive >> mInputChannel;
	text_archive >> mArbitrationBitRate;
	text_archive >> mInverted;

	text_archive >> value ;
  mProtocol = ProtocolSetting (value) ;

	text_archive >> value ;
  mSimulatorGeneratedAckSlot = SimulatorGeneratedBit (value) ;

	text_archive >> value ;
  mSimulatorGeneratedFrameType = SimulatorGeneratedFrameType (value) ;

	text_archive >> value ;
  mSimulatorGeneratedESISlot = SimulatorGeneratedBit (value) ;

	ClearChannels();
	AddChannel( mInputChannel, "CANFD (Molinaro)", true );

	UpdateInterfacesFromSettings();
}

//--------------------------------------------------------------------------------------------------

const char* CANFDMolinaroAnalyzerSettings::SaveSettings () {
	SimpleArchive text_archive;

	text_archive << mInputChannel;
	text_archive << mArbitrationBitRate;
	text_archive << mInverted;
	text_archive << U32 (mProtocol) ;
	text_archive << U32 (mSimulatorGeneratedAckSlot) ;
	text_archive << U32 (mSimulatorGeneratedFrameType) ;
	text_archive << U32 (mSimulatorGeneratedESISlot) ;

	return SetReturnString (text_archive.GetString ()) ;
}

//--------------------------------------------------------------------------------------------------
