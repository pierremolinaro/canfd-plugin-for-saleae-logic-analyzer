#ifndef CANMOLINARO_SIMULATION_DATA_GENERATOR
#define CANMOLINARO_SIMULATION_DATA_GENERATOR

//--------------------------------------------------------------------------------------------------

#include <SimulationChannelDescriptor.h>
#include <string>

//--------------------------------------------------------------------------------------------------

class CANFDMolinaroAnalyzerSettings;

//--------------------------------------------------------------------------------------------------

typedef enum {ACK_SLOT_DOMINANT, ACK_SLOT_RECESSIVE} AckSlot ;

//--------------------------------------------------------------------------------------------------

class CANMolinaroSimulationDataGenerator {
public:
  CANMolinaroSimulationDataGenerator();
  ~CANMolinaroSimulationDataGenerator();

  void Initialize ( U32 simulation_sample_rate, CANFDMolinaroAnalyzerSettings* settings );
  U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channel );

protected:
  CANFDMolinaroAnalyzerSettings* mSettings;
  U32 mSimulationSampleRateHz;

//---------------- Pseudo Random Generator
//https://stackoverflow.com/questions/15500621/c-c-algorithm-to-produce-same-pseudo-random-number-sequences-from-same-seed-on
  protected: U32 mSeed ;
  protected: U32 pseudoRandomValue (void) {
    mSeed = 8253729 * mSeed + 2396403 ;
    return mSeed ;
  }

protected: void createCANFrame (const U32 inSamplesPerArbitrationBit,
                                const bool inInverted) ;

protected: void createBaseCANFrame (const U32 inSamplesPerArbitrationBit,
                                    const bool inInverted,
                                    const AckSlot inAck,
                                    const bool inExtended,
                                    const bool inRemote) ;

protected: void createCANFD_Frame (const U32 inSamplesPerArbitrationBit,
                                   const bool in_canfd_24_64,
                                   const U32 inSamplesPerDataBit,
                                   const bool inInverted,
                                   const AckSlot inAck,
                                   const bool inExtended) ;

protected: SimulationChannelDescriptor mSerialSimulationData;

} ;

//--------------------------------------------------------------------------------------------------

#endif //CANMOLINARO_SIMULATION_DATA_GENERATOR
