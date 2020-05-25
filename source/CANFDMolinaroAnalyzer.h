#ifndef CANFDMOLINARO_ANALYZER_H
#define CANFDMOLINARO_ANALYZER_H

#include <Analyzer.h>
#include "CANFDMolinaroAnalyzerResults.h"
#include "CANFDMolinaroSimulationDataGenerator.h"

class CANFDMolinaroAnalyzerSettings;
class ANALYZER_EXPORT CANFDMolinaroAnalyzer : public Analyzer2
{
public:
	CANFDMolinaroAnalyzer();
	virtual ~CANFDMolinaroAnalyzer();

	virtual void SetupResults();
	virtual void WorkerThread();

	virtual U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channels );
	virtual U32 GetMinimumSampleRateHz();

	virtual const char* GetAnalyzerName() const;
	virtual bool NeedsRerun();

protected: //vars
	std::auto_ptr< CANFDMolinaroAnalyzerSettings > mSettings;
	std::auto_ptr< CANFDMolinaroAnalyzerResults > mResults;
	AnalyzerChannelData* mSerial;

	CANFDMolinaroSimulationDataGenerator mSimulationDataGenerator;
	bool mSimulationInitilized;

	//Serial analysis vars:
	U32 mSampleRateHz;
	U32 mStartOfStopBitOffset;
	U32 mEndOfStopBitOffset;
};

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer( );
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

#endif //CANFDMOLINARO_ANALYZER_H
