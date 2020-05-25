#ifndef CANFDMOLINARO_ANALYZER_SETTINGS
#define CANFDMOLINARO_ANALYZER_SETTINGS

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

class CANFDMolinaroAnalyzerSettings : public AnalyzerSettings
{
public:
	CANFDMolinaroAnalyzerSettings();
	virtual ~CANFDMolinaroAnalyzerSettings();

	virtual bool SetSettingsFromInterfaces();
	void UpdateInterfacesFromSettings();
	virtual void LoadSettings( const char* settings );
	virtual const char* SaveSettings();

	
	Channel mInputChannel;
	U32 mBitRate;

protected:
	std::auto_ptr< AnalyzerSettingInterfaceChannel >	mInputChannelInterface;
	std::auto_ptr< AnalyzerSettingInterfaceInteger >	mBitRateInterface;
};

#endif //CANFDMOLINARO_ANALYZER_SETTINGS
