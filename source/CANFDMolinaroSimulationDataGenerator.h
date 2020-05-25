#ifndef CANFDMOLINARO_SIMULATION_DATA_GENERATOR
#define CANFDMOLINARO_SIMULATION_DATA_GENERATOR

#include <SimulationChannelDescriptor.h>
#include <string>
class CANFDMolinaroAnalyzerSettings;

class CANFDMolinaroSimulationDataGenerator
{
public:
	CANFDMolinaroSimulationDataGenerator();
	~CANFDMolinaroSimulationDataGenerator();

	void Initialize( U32 simulation_sample_rate, CANFDMolinaroAnalyzerSettings* settings );
	U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channel );

protected:
	CANFDMolinaroAnalyzerSettings* mSettings;
	U32 mSimulationSampleRateHz;

protected:
	void CreateSerialByte();
	std::string mSerialText;
	U32 mStringIndex;

	SimulationChannelDescriptor mSerialSimulationData;

};
#endif //CANFDMOLINARO_SIMULATION_DATA_GENERATOR