#include "CANFDMolinaroSimulationDataGenerator.h"
#include "CANFDMolinaroAnalyzerSettings.h"

//--------------------------------------------------------------------------------------------------

#include <AnalyzerHelpers.h>

//--------------------------------------------------------------------------------------------------
//  CAN 2.0B FRAME GENERATOR
//--------------------------------------------------------------------------------------------------

typedef enum {standardFrame, extendedFrame} FrameFormat ;

//--------------------------------------------------------------------------------------------------

typedef enum {dataFrame, remoteFrame} FrameType ;

//--------------------------------------------------------------------------------------------------

typedef enum {DOMINANT_BIT, RECESSIVE_BIT} GeneratedBit ;

//--------------------------------------------------------------------------------------------------

static const uint32_t CAN_FRAME_MAX_LENGTH = 160 ;

//--------------------------------------------------------------------------------------------------

class CANFrameBitsGenerator {
  public : CANFrameBitsGenerator (const uint32_t inIdentifier,
                                  const FrameFormat inFrameFormat,
                                  const uint8_t inDataLength,
                                  const uint8_t inData [8],
                                  const FrameType inFrameType,
                                  const AckSlot inAckSlot) ;

//--- Public methods
  public : inline uint32_t frameLength (void) const { return mFrameLength ; }
  public : bool bitAtIndex (const uint32_t inIndex) const ;

//--- Private methods (used during frame generation)
  private: void enterBitAppendStuff (const bool inBit) ;

  private: void enterBitNoStuff (const bool inBit) ;

//--- Private properties
  private: uint32_t mBits [5] ;
  private: uint8_t mFrameLength ;

//--- CRC computation
  private : uint32_t mConsecutiveBitCount ;
  private : bool mLastBitValue ;
  private : uint16_t mCRCAccumulator ;
} ;

//--------------------------------------------------------------------------------------------------
CANFrameBitsGenerator::CANFrameBitsGenerator (const uint32_t inIdentifier,
                                              const FrameFormat inFrameFormat,
                                              const uint8_t inDataLength,
                                              const uint8_t inData [8],
                                              const FrameType inFrameType,
                                              const AckSlot inAckSlot) :
mBits (),
mFrameLength (0),
mConsecutiveBitCount (1),
mLastBitValue (true),
mCRCAccumulator (0) {
  for (uint32_t i=0 ; i<5 ; i++) {
    mBits [i] = UINT32_MAX ;
  }
  const uint8_t dataLength = (inDataLength > 15) ? 15 : inDataLength ;
//--- Generate frame
  enterBitAppendStuff (false) ; // SOF
  switch (inFrameFormat) {
  case extendedFrame :
    for (uint8_t idx = 28 ; idx >= 18 ; idx--) { // Identifier
      const bool bit = (inIdentifier & (1 << idx)) != 0 ;
      enterBitAppendStuff (bit) ;
    }
    enterBitAppendStuff (true) ; // SRR
    enterBitAppendStuff (true) ; // IDE
    for (int idx = 17 ; idx >= 0 ; idx--) { // Identifier
      const bool bit = (inIdentifier & (1 << idx)) != 0 ;
      enterBitAppendStuff (bit) ;
    }
    break ;
  case standardFrame :
    for (int idx = 10 ; idx >= 0 ; idx--) { // Identifier
      const bool bit = (inIdentifier & (1 << idx)) != 0 ;
      enterBitAppendStuff (bit) ;
    }
    break ;
  }
  enterBitAppendStuff (inFrameType == remoteFrame) ; // RTR
  enterBitAppendStuff (false) ; // RESERVED 1
  enterBitAppendStuff (false) ; // RESERVED 0
  enterBitAppendStuff ((dataLength & 8) != 0) ; // DLC 3
  enterBitAppendStuff ((dataLength & 4) != 0) ; // DLC 2
  enterBitAppendStuff ((dataLength & 2) != 0) ; // DLC 1
  enterBitAppendStuff ((dataLength & 1) != 0) ; // DLC 0
//--- Enter DATA
  if (inFrameType == dataFrame) {
    const uint8_t maxLength = (dataLength > 8) ? 8 : dataLength ;
    for (uint8_t dataIdx = 0 ; dataIdx < maxLength ; dataIdx ++) {
      for (int bitIdx = 7 ; bitIdx >= 0 ; bitIdx--) {
        enterBitAppendStuff ((inData [dataIdx] & (1 << bitIdx)) != 0) ;
      }
    }
  }
//--- Enter CRC SEQUENCE
  const uint16_t frameCRC = mCRCAccumulator ;
  for (int idx = 14 ; idx >= 0 ; idx--) {
    const bool bit = (frameCRC & (1 << idx)) != 0 ;
    enterBitAppendStuff (bit) ;
  }
//--- Enter ACK, EOF, INTERMISSION
  enterBitNoStuff (true) ; // CRC DEL
  switch (inAckSlot) {
  case ACK_SLOT_DOMINANT :
    enterBitNoStuff (false) ;
    break ;
  case ACK_SLOT_RECESSIVE :
    enterBitNoStuff (true) ;
    break ;
  }
//--- ACK DEL, EOF (7), INTERMISSION (3), all RECESSIVE
  mFrameLength += 11 ;
}

//--------------------------------------------------------------------------------------------------

void CANFrameBitsGenerator::enterBitAppendStuff (const bool inBit) {
//--- Compute CRC
  const bool bit14 = (mCRCAccumulator & (1 << 14)) != 0 ;
  const bool crc_nxt = inBit ^ bit14 ;
  mCRCAccumulator <<= 1 ;
  mCRCAccumulator &= 0x7FFF ;
  if (crc_nxt) {
    mCRCAccumulator ^= 0x4599 ;
  }
//--- Emit bit
  if (!inBit) {
    const uint32_t idx = mFrameLength / 32 ;
    const uint32_t offset = mFrameLength % 32 ;
    mBits [idx] &= ~ (1U << offset) ;
  }
  mFrameLength ++ ;
//--- Add a stuff bit ?
  if (mLastBitValue == inBit) {
    mConsecutiveBitCount += 1 ;
    if (mConsecutiveBitCount == 5) {
      mLastBitValue ^= true ;
      if (!mLastBitValue) {
        const uint32_t idx = mFrameLength / 32 ;
        const uint32_t offset = mFrameLength % 32 ;
        mBits [idx] &= ~ (1U << offset) ;
      }
      mFrameLength ++ ;
      mConsecutiveBitCount = 1 ;
    }
  }else{
    mLastBitValue = inBit ;
    mConsecutiveBitCount = 1 ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFrameBitsGenerator::enterBitNoStuff (const bool inBit) {
//--- Emit bit
  if (!inBit) {
    const uint32_t idx = mFrameLength / 32 ;
    const uint32_t offset = mFrameLength % 32 ;
    mBits [idx] &= ~ (1U << offset) ;
  }
  mFrameLength ++ ;
}

//--------------------------------------------------------------------------------------------------

bool CANFrameBitsGenerator::bitAtIndex (const uint32_t inIndex) const {
  bool result = true ; // RECESSIF
  if (inIndex < mFrameLength) {
    const uint32_t idx = inIndex / 32 ;
    const uint32_t offset = inIndex % 32 ;
    result = (mBits [idx] & (1U << offset)) != 0 ;
  }
  return result ;
}

//--------------------------------------------------------------------------------------------------
//  CANFD FRAME GENERATOR
//--------------------------------------------------------------------------------------------------

class CANFDFrameBitsGenerator {
  public : CANFDFrameBitsGenerator (const uint32_t inIdentifier,
                                    const FrameFormat inFrameFormat,
                                    const ProtocolSetting inProtocolType,
                                    const uint8_t inDataLength,
                                    const GeneratedBit inBSR,
                                    const uint8_t inData [64],
                                    const AckSlot inAckSlot,
                                    const GeneratedBit inESISlot) ;

//--- Public methods
  public: inline uint8_t dataLengthCode (void) const { return mDataLengthCode ; }
  public: inline uint8_t dataAtIndex (const uint32_t inIndex) const { return mData [inIndex] ; }
  public: inline uint32_t identifier (void) const { return mIdentifier ; }
  public: inline uint32_t frameLength (void) const { return mFrameLength ; }
  public: inline uint32_t stuffBitCount (void) const { return mStuffBitCount ; }
  public: inline uint32_t frameCRC (void) const { return mFrameCRC ; }
  public: bool bitAtIndex (const uint32_t inIndex) const ;
  public: bool dataBitRateAtIndex (const uint32_t inIndex) const ;

//--- Private methods (used during frame generation)
  private: void enterBitComputeCRCAppendStuff (const bool inBit, const bool inUseDataBitRate) ;

  private: void enterBitInFrame (const bool inBit, const bool inUseDataBitRate) ;

  private: void enterBitInFrameComputeCRC (const bool inBit, const bool inUseDataBitRate) ;

   public: static uint8_t lengthForCode (const uint8_t inDataLengthCode) ;

//--- Private properties
  private: uint32_t mBits [30] ;
  private: uint32_t mDataRateBits [30] ;
  private: uint8_t mData [64] ;
  private: const uint32_t mIdentifier ;
  private: uint32_t mFrameCRC ;
  private: uint32_t mFrameLength ;
  private: uint32_t mCRCAccumulator17 ;
  private: uint32_t mCRCAccumulator21 ;
  private: uint8_t mStuffBitCount ;
  private: const uint8_t mDataLengthCode ;
  private: const FrameFormat mFrameFormat ;
  private: const ProtocolSetting mProtocolType ;
  private: const AckSlot mAckSlot ;

  private: bool mLastBitValue ;
  private: uint8_t mConsecutiveBitCount ;
} ;

//--------------------------------------------------------------------------------------------------

CANFDFrameBitsGenerator::CANFDFrameBitsGenerator (const uint32_t inIdentifier,
                                                  const FrameFormat inFrameFormat,
                                                  const ProtocolSetting inProtocolType,
                                                  const uint8_t inDataLengthCode,
                                                  const GeneratedBit inBSR,
                                                  const uint8_t inData [64],
                                                  const AckSlot inAckSlot,
                                                  const GeneratedBit inESISlot) :
mBits (),
mDataRateBits (),
mData (),
mIdentifier (inIdentifier),
mFrameCRC (0),
mFrameLength (0),
mCRCAccumulator17 (0),
mCRCAccumulator21 (0),
mStuffBitCount (0),
mDataLengthCode (inDataLengthCode),
mFrameFormat (inFrameFormat),
mProtocolType (inProtocolType),
mAckSlot (inAckSlot),
mLastBitValue (true),
mConsecutiveBitCount (1) {
  for (uint32_t i=0 ; i<30 ; i++) {
    mBits [i] = UINT32_MAX ; // By default, all bits are recessive
    mDataRateBits [i] = 0 ; // By default, all bits in Arbitration bit rate
  }
  const uint8_t dataByteCount = CANFDFrameBitsGenerator::lengthForCode (mDataLengthCode) ;
  for (uint8_t i=0 ; i<dataByteCount ; i++) {
    mData [i] = inData [i] ;
  }
//--- Change CRC initial value according to ISO
  switch (mProtocolType) {
  case CANFD_NON_ISO_PROTOCOL :
    break ;
  case CANFD_ISO_PROTOCOL :
    mCRCAccumulator17 = 1U << 16 ;
    mCRCAccumulator21 = 1U << 20 ;
    break ;
  }
//--- Enter SOF
  enterBitComputeCRCAppendStuff (false, false) ;
//--- Enter Identifier
  switch (mFrameFormat) {
  case FrameFormat::extendedFrame :
    for (uint8_t idx = 28 ; idx >= 18 ; idx--) { // Identifier
      const bool bit = (mIdentifier & (1 << idx)) != 0 ;
      enterBitComputeCRCAppendStuff (bit, false) ;
    }
    enterBitComputeCRCAppendStuff (true, false) ; // SRR
    enterBitComputeCRCAppendStuff (true, false) ; // IDE
    for (int idx = 17 ; idx >= 0 ; idx--) { // Identifier
      const bool bit = (mIdentifier & (1 << idx)) != 0 ;
      enterBitComputeCRCAppendStuff (bit, false) ;
    }
    break ;
  case FrameFormat::standardFrame :
    for (int idx = 10 ; idx >= 0 ; idx--) { // Identifier
      const bool bit = (mIdentifier & (1 << idx)) != 0 ;
      enterBitComputeCRCAppendStuff (bit, false) ;
    }
    enterBitComputeCRCAppendStuff (false, false) ; // R1
    break ;
  }
//--- Enter DLC
  enterBitComputeCRCAppendStuff (false, false) ; // IDE
  enterBitComputeCRCAppendStuff (true, false) ; // FDF
  enterBitComputeCRCAppendStuff (false, false) ; // R0
  const bool dataBitRate = inBSR == RECESSIVE_BIT ;
  enterBitComputeCRCAppendStuff (dataBitRate, dataBitRate) ; // BRS
  switch (inESISlot) {
  case DOMINANT_BIT:
    enterBitComputeCRCAppendStuff (false, dataBitRate) ; // ESI
    break ;
  case RECESSIVE_BIT:
    enterBitComputeCRCAppendStuff (true, dataBitRate) ; // ESI
    break ;
  }
  enterBitComputeCRCAppendStuff ((mDataLengthCode & 8) != 0, dataBitRate) ;
  enterBitComputeCRCAppendStuff ((mDataLengthCode & 4) != 0, dataBitRate) ;
  enterBitComputeCRCAppendStuff ((mDataLengthCode & 2) != 0, dataBitRate) ;
  if (dataByteCount == 0) {
   enterBitInFrameComputeCRC ((mDataLengthCode & 1) != 0, dataBitRate) ;
  }else{
   enterBitComputeCRCAppendStuff ((mDataLengthCode & 1) != 0, dataBitRate) ;
  }
//--- Enter DATA
  bool lastBit = mLastBitValue ;
  for (uint8_t dataIdx = 0 ; dataIdx < dataByteCount ; dataIdx ++) {
    for (int bitIdx = 7 ; bitIdx >= 0 ; bitIdx--) {
      lastBit = (inData [dataIdx] & (1 << bitIdx)) != 0 ;
      if ((dataIdx == (dataByteCount - 1)) && (bitIdx == 0)) { // Last data bit
        enterBitInFrameComputeCRC (lastBit, dataBitRate) ;
      }else{
        enterBitComputeCRCAppendStuff (lastBit, dataBitRate) ;
      }
    }
  }
//--- Enter STUFF BIT COUNT
  switch (mProtocolType) {
  case CANFD_NON_ISO_PROTOCOL :
    break ;
  case CANFD_ISO_PROTOCOL :
    { enterBitInFrame (!lastBit, dataBitRate) ;
      const uint8_t GRAY_CODE_PARITY [8] = {0, 3, 6, 5, 12, 15, 10, 9} ;
      const uint8_t code = GRAY_CODE_PARITY [mStuffBitCount % 8] ;
      enterBitInFrameComputeCRC ((code & 8) != 0, dataBitRate) ;
      enterBitInFrameComputeCRC ((code & 4) != 0, dataBitRate) ;
      enterBitInFrameComputeCRC ((code & 2) != 0, dataBitRate) ;
      lastBit = (code & 1) != 0 ;
      enterBitInFrameComputeCRC (lastBit, dataBitRate) ;
    }
    break ;
  }
//--- Enter CRC SEQUENCE
  enterBitInFrame (!lastBit, dataBitRate) ;
  const uint32_t frameCRC = (mDataLengthCode > 10) ? mCRCAccumulator21 : mCRCAccumulator17 ;
  mFrameCRC = frameCRC ;
  const int crcFirstBitIndex = (mDataLengthCode > 10) ? 20 : 16 ;
  uint32_t bitCount = 0 ;
  for (int idx = crcFirstBitIndex ; idx >= 0 ; idx--) {
    const bool crc_bit = (frameCRC & (1 << idx)) != 0 ;
    enterBitInFrame (crc_bit, dataBitRate) ;
    bitCount += 1 ;
    if (bitCount == 4) {
      bitCount = 0 ;
      enterBitInFrame (!crc_bit, dataBitRate) ;
    }
  }
  enterBitInFrame (true, false) ; // CRC DEL (arbitration bit rate)
//--- Enter ACK, EOF, INTERMISSION
  switch (mAckSlot) {
  case ACK_SLOT_DOMINANT :
    enterBitInFrame (false, false) ;
    break ;
  case ACK_SLOT_RECESSIVE :
    enterBitInFrame (true, false) ;
    break ;
  }
  enterBitInFrame (true, false) ;
  for (uint8_t i=0 ; i<7 ; i++) {
    enterBitInFrame (true, false) ;
  }
  for (uint8_t i=0 ; i<3 ; i++) {
    enterBitInFrame (true, false) ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDFrameBitsGenerator::enterBitInFrameComputeCRC (const bool inBit, const bool inUseDataBitRate) {
//--- Enter bit in frame
  enterBitInFrame (inBit, inUseDataBitRate) ;
//--- Enter in CRC17
  const bool bit16 = (mCRCAccumulator17 & (1U << 16)) != 0 ;
  const bool crc17_nxt = inBit ^ bit16 ;
  mCRCAccumulator17 <<= 1 ;
  mCRCAccumulator17 &= 0x1FFFF ;
  if (crc17_nxt) {
    mCRCAccumulator17 ^= 0x1685B ;
  }
//--- Enter in CRC21
  const bool bit20 = (mCRCAccumulator21 & (1U << 20)) != 0 ;
  const bool crc21_nxt = inBit ^ bit20 ;
  mCRCAccumulator21 <<= 1 ;
  mCRCAccumulator21 &= 0x1FFFFF ;
  if (crc21_nxt) {
    mCRCAccumulator21 ^= 0x102899 ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDFrameBitsGenerator::enterBitComputeCRCAppendStuff (const bool inBit, const bool inUseDataBitRate) {
//--- Enter bit in frame
  enterBitInFrameComputeCRC (inBit, inUseDataBitRate) ;
//--- Add a stuff bit ?
  if (mLastBitValue == inBit) {
    mConsecutiveBitCount += 1 ;
    if (mConsecutiveBitCount == 5) {
      mConsecutiveBitCount = 1 ;
      mStuffBitCount += 1 ;
      mLastBitValue ^= true ;
      enterBitInFrameComputeCRC (mLastBitValue, inUseDataBitRate) ;
    }
  }else{
    mLastBitValue = inBit ;
    mConsecutiveBitCount = 1 ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDFrameBitsGenerator::enterBitInFrame (const bool inBit, const bool inUseDataBitRate) {
  const uint32_t idx = mFrameLength / 32 ;
  const uint32_t offset = mFrameLength % 32 ;
  if (!inBit) {
    mBits [idx] &= ~ (1U << offset) ;
  }
  if (inUseDataBitRate) {
    mDataRateBits [idx] |= (1U << offset) ;
  }
  mFrameLength += 1 ;
}

//--------------------------------------------------------------------------------------------------

bool CANFDFrameBitsGenerator::bitAtIndex (const uint32_t inIndex) const {
  bool result = true ; // RECESSIF
  if (inIndex < mFrameLength) {
    const uint32_t idx = inIndex / 32 ;
    const uint32_t offset = inIndex % 32 ;
    result = (mBits [idx] & (1U << offset)) != 0 ;
  }
  return result ;
}

//--------------------------------------------------------------------------------------------------

bool CANFDFrameBitsGenerator::dataBitRateAtIndex (const uint32_t inIndex) const {
  bool result = false ;
  if (inIndex < mFrameLength) {
    const uint32_t idx = inIndex / 32 ;
    const uint32_t offset = inIndex % 32 ;
    result = (mDataRateBits [idx] & (1U << offset)) != 0 ;
  }
  return result ;
}

//--------------------------------------------------------------------------------------------------

uint8_t CANFDFrameBitsGenerator::lengthForCode (const uint8_t inDataLengthCode) {
  const uint8_t LENGTH [16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64} ;
  return LENGTH [inDataLengthCode] ;
}

//--------------------------------------------------------------------------------------------------
//  CANMolinaroSimulationDataGenerator
//--------------------------------------------------------------------------------------------------

CANMolinaroSimulationDataGenerator::CANMolinaroSimulationDataGenerator() {
}

//--------------------------------------------------------------------------------------------------

CANMolinaroSimulationDataGenerator::~CANMolinaroSimulationDataGenerator () {
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroSimulationDataGenerator::Initialize(const U32 simulation_sample_rate,
                                                    CANFDMolinaroAnalyzerSettings * settings) {
  mSimulationSampleRateHz = simulation_sample_rate;
  mSettings = settings;

  mSerialSimulationData.SetChannel (mSettings->mInputChannel);
  mSerialSimulationData.SetSampleRate (simulation_sample_rate) ;
  mSerialSimulationData.SetInitialBitState (BIT_HIGH) ;
}

//--------------------------------------------------------------------------------------------------

U32 CANMolinaroSimulationDataGenerator::GenerateSimulationData (const U64 largest_sample_requested,
                                                                const U32 sample_rate,
                                        SimulationChannelDescriptor** simulation_channel) {
  const U64 adjusted_largest_sample_requested = AnalyzerHelpers::AdjustSimulationTargetSample (
    largest_sample_requested,
    sample_rate,
    mSimulationSampleRateHz
  );

 //--- Random Seed
  mSeed = 0 ; // mSettings->simulatorRandomSeed ()) ;

//--- Let's move forward for 11 recessive bits
  const U32 samplesPerArbitrationBitRate = mSimulationSampleRateHz / mSettings->arbitrationBitRate () ;
  const bool inverted = mSettings->inverted () ;
  mSerialSimulationData.TransitionIfNeeded (inverted ? BIT_LOW : BIT_HIGH) ;  // Edge for IDLE
  mSerialSimulationData.Advance (samplesPerArbitrationBitRate * 11) ;
  mSerialSimulationData.TransitionIfNeeded (inverted ? BIT_HIGH : BIT_LOW) ;  // Edge for SOF bit

  while (mSerialSimulationData.GetCurrentSampleNumber() < adjusted_largest_sample_requested) {
    createCANFrame (samplesPerArbitrationBitRate, inverted) ;
  }

  *simulation_channel = &mSerialSimulationData;
  return 1;
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroSimulationDataGenerator::createCANFrame (const U32 inSamplesPerArbitrationBit,
                                                         const bool inInverted) {
  const U32 samplesPerDataBit = mSimulationSampleRateHz / mSettings->dataBitRate () ;
  const SimulatorGeneratedFrameType frameTypes = mSettings->generatedFrameType () ;
//--- Select Frame type to generate
  bool canFD_frame = false ;
  bool canfd_24_64 = false ;
  bool extended = false ;
  bool remoteFrame = false ;
  switch (frameTypes) {
  case GENERATE_ALL_FRAME_TYPES :
    extended = (pseudoRandomValue () & 1) != 0 ;
    remoteFrame = (pseudoRandomValue () & 1) != 0 ;
    canFD_frame = (pseudoRandomValue () & 1) != 0 ;
    canfd_24_64 = (pseudoRandomValue () & 1) != 0 ;
    break ;
  case GENERATE_ONLY_STANDARD_DATA :
    break ;
  case GENERATE_ONLY_EXTENDED_DATA :
    extended = true ;
    break ;
  case GENERATE_ONLY_STANDARD_REMOTE :
    remoteFrame = true ;
    break ;
  case GENERATE_ONLY_EXTENDED_REMOTE :
    extended = true ;
    remoteFrame = true ;
    break ;
  case GENERATE_ONLY_CANFD_BASE_0_16 :
    canFD_frame = true ;
    break ;
  case GENERATE_ONLY_CANFD_EXTENDED_0_16 :
    canFD_frame = true ;
    extended = true ;
    break ;
  case GENERATE_ONLY_CANFD_BASE_20_64 :
    canFD_frame = true ;
    canfd_24_64 = true ;
    break ;
  case GENERATE_ONLY_CANFD_EXTENDED_20_64 :
    canFD_frame = true ;
    canfd_24_64 = true ;
    extended = true ;
    break ;
  }
//--- Select ACK SLOT level
  AckSlot ack = AckSlot::ACK_SLOT_DOMINANT ;
  switch (mSettings->generatedAckSlot ()) {
  case GENERATE_BIT_DOMINANT :
    break ;
  case GENERATE_BIT_RECESSIVE :
    ack = AckSlot::ACK_SLOT_RECESSIVE ;
    break ;
  case GENERATE_BIT_RANDOMLY :
    ack = ((pseudoRandomValue () & 1) != 0) ? AckSlot::ACK_SLOT_DOMINANT : AckSlot::ACK_SLOT_RECESSIVE ;
    break ;
  }
//--- Generate CANFD Frame, 0 to 16 bytes
  if (canFD_frame) {
    createCANFD_Frame (inSamplesPerArbitrationBit, canfd_24_64, samplesPerDataBit, inInverted, ack, extended) ;
  }else{
    createBaseCANFrame (inSamplesPerArbitrationBit, inInverted, ack, extended, remoteFrame) ;
  }
//--- We need to end recessive
  mSerialSimulationData.TransitionIfNeeded (inInverted ? BIT_LOW : BIT_HIGH) ;
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroSimulationDataGenerator::createCANFD_Frame (const U32 inSamplesPerArbitrationBit,
                                                            const bool in_canfd_24_64,
                                                            const U32 inSamplesPerDataBit,
                                                            const bool inInverted,
                                                            const AckSlot inAck,
                                                            const bool inExtended) {
//--- Select BSR level
  GeneratedBit bsr = GeneratedBit::DOMINANT_BIT ;
  switch (mSettings->generatedBSRSlot ()) {
  case GENERATE_BIT_DOMINANT :
    break ;
  case GENERATE_BIT_RECESSIVE :
    bsr = GeneratedBit::RECESSIVE_BIT ;
    break ;
  case GENERATE_BIT_RANDOMLY :
    bsr = ((pseudoRandomValue () & 1) != 0) ? GeneratedBit::DOMINANT_BIT : GeneratedBit::RECESSIVE_BIT ;
    break ;
  }
//--- Select ESI level
  GeneratedBit esi = GeneratedBit::DOMINANT_BIT ;
  switch (mSettings->generatedESISlot ()) {
  case GENERATE_BIT_DOMINANT :
    break ;
  case GENERATE_BIT_RECESSIVE :
    esi = GeneratedBit::RECESSIVE_BIT ;
    break ;
  case GENERATE_BIT_RANDOMLY :
    esi = ((pseudoRandomValue () & 1) != 0) ? GeneratedBit::DOMINANT_BIT : GeneratedBit::RECESSIVE_BIT ;
    break ;
  }
//---
  uint8_t data [64] ;
  const FrameFormat format = inExtended ? FrameFormat::extendedFrame : FrameFormat::standardFrame ;
  const uint32_t identifier = uint32_t (pseudoRandomValue ()) & (inExtended ? 0x1FFFFFFF : 0x7FF) ;
  const uint8_t dataLengthCode = in_canfd_24_64
    ? (uint8_t (pseudoRandomValue ()) % 5 + 11) // 11, ..., 15
    : (uint8_t (pseudoRandomValue ()) % 11) ;   // 0 ... 105
  for (uint32_t i=0 ; i<CANFDFrameBitsGenerator::lengthForCode (dataLengthCode) ; i++) {
    data [i] = uint8_t (pseudoRandomValue ()) ;
  }
  const ProtocolSetting protocol = mSettings->protocol () ;
  const CANFDFrameBitsGenerator frame (identifier, format, protocol, dataLengthCode, bsr, data, inAck, esi) ;
//--- Now, send FD frame
  bool bitRateIsDataBitRate = false ;
  for (U32 i=0 ; i < frame.frameLength () ; i++) {
    const bool bit = frame.bitAtIndex (i) ^ inInverted ;
    const bool dataBitRate = frame.dataBitRateAtIndex (i) ;
    mSerialSimulationData.TransitionIfNeeded (bit ? BIT_HIGH : BIT_LOW) ;
    if (bitRateIsDataBitRate == dataBitRate) {
      mSerialSimulationData.Advance (dataBitRate ? inSamplesPerDataBit : inSamplesPerArbitrationBit) ;
    }else{
      mSerialSimulationData.Advance ((inSamplesPerDataBit + inSamplesPerArbitrationBit) / 2) ;
    }
    bitRateIsDataBitRate = dataBitRate ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroSimulationDataGenerator::createBaseCANFrame (const U32 inSamplesPerArbitrationBit,
                                                             const bool inInverted,
                                                             const AckSlot inAck,
                                                             const bool inExtended,
                                                             const bool inRemote) {
//----
  uint8_t data [8] ;
  const FrameFormat format = inExtended ? FrameFormat::extendedFrame : FrameFormat::standardFrame ;
  const FrameType type = inRemote ? FrameType::remoteFrame : FrameType::dataFrame ;
  const uint32_t identifier = uint32_t (pseudoRandomValue ()) & (inExtended ? 0x1FFFFFFF : 0x7FF) ;
  const uint8_t dataLength = uint8_t (pseudoRandomValue ()) % 9 ;
  if (! remoteFrame) {
    for (uint32_t i=0 ; i<dataLength ; i++) {
      data [i] = uint8_t (pseudoRandomValue ()) ;
    }
  }
  const CANFrameBitsGenerator frame (identifier, format, dataLength, data, type, inAck) ;
//--- Generated bit error index
//   uint8_t generatedErrorBitIndex = uint8_t (uint32_t (pseudoRandomValue ()) % frame.frameLength ()) ;
  pseudoRandomValue () ; // For compatibility witrh CAN 2.0B generator
//--- Now, send frame
  for (U32 i=0 ; i < frame.frameLength () ; i++) {
    const bool bit = frame.bitAtIndex (i) ^ inInverted ;
    mSerialSimulationData.TransitionIfNeeded (bit ? BIT_HIGH : BIT_LOW) ;
    mSerialSimulationData.Advance (inSamplesPerArbitrationBit) ;
  }
}

//--------------------------------------------------------------------------------------------------
