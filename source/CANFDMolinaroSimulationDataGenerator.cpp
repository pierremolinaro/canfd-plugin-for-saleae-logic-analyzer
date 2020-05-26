#include "CANFDMolinaroSimulationDataGenerator.h"
#include "CANFDMolinaroAnalyzerSettings.h"

//--------------------------------------------------------------------------------------------------

#include <AnalyzerHelpers.h>

//--------------------------------------------------------------------------------------------------
//  CAN 2.0B FRAME GENERATOR
//--------------------------------------------------------------------------------------------------

typedef enum {standard, extended} FrameFormat ;

//--------------------------------------------------------------------------------------------------

typedef enum {data, remote} FrameType ;

//--------------------------------------------------------------------------------------------------

typedef enum {ACK_SLOT_DOMINANT, ACK_SLOT_RECESSIVE} AckSlot ;

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
  private : uint32_t mConsecutiveBitCount = 1 ;
  private : bool mLastBitValue = true ;
  private : uint16_t mCRCAccumulator = 0 ;
} ;

//--------------------------------------------------------------------------------------------------

CANFrameBitsGenerator::CANFrameBitsGenerator (const uint32_t inIdentifier,
                                              const FrameFormat inFrameFormat,
                                              const uint8_t inDataLength,
                                              const uint8_t inData [8],
                                              const FrameType inFrameType,
                                              const AckSlot inAckSlot) :
mBits (),
mFrameLength (0) {
  for (uint32_t i=0 ; i<5 ; i++) {
    mBits [i] = UINT32_MAX ;
  }
  const uint8_t dataLength = (inDataLength > 15) ? 15 : inDataLength ;
//--- Generate frame
  enterBitAppendStuff (false) ; // SOF
  switch (inFrameFormat) {
  case FrameFormat::extended :
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
  case FrameFormat::standard :
    for (int idx = 10 ; idx >= 0 ; idx--) { // Identifier
      const bool bit = (inIdentifier & (1 << idx)) != 0 ;
      enterBitAppendStuff (bit) ;
    }
    break ;
  }
  enterBitAppendStuff (inFrameType == FrameType::remote) ; // RTR
  enterBitAppendStuff (false) ; // RESERVED 1
  enterBitAppendStuff (false) ; // RESERVED 0
  enterBitAppendStuff ((dataLength & 8) != 0) ; // DLC 3
  enterBitAppendStuff ((dataLength & 4) != 0) ; // DLC 2
  enterBitAppendStuff ((dataLength & 2) != 0) ; // DLC 1
  enterBitAppendStuff ((dataLength & 1) != 0) ; // DLC 0
//--- Enter DATA
  if (inFrameType == FrameType::data) {
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
  case AckSlot::ACK_SLOT_DOMINANT :
    enterBitNoStuff (false) ;
    break ;
  case AckSlot::ACK_SLOT_RECESSIVE :
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

typedef enum {PROTOCOL_ISO, PROTOCOL_NON_ISO} ProtocolType ;

//--------------------------------------------------------------------------------------------------

class CANFDFrameBitsGenerator {
  public : CANFDFrameBitsGenerator (const uint32_t inIdentifier,
                                    const FrameFormat inFrameFormat,
                                    const ProtocolType inProtocolType,
                                    const uint8_t inDataLength,
                                    const uint8_t inData [64],
                                    const AckSlot inAckSlot) ;

//--- Public methods
  public: inline uint8_t dataLengthCode (void) const { return mDataLengthCode ; }
  public: inline uint8_t dataAtIndex (const uint32_t inIndex) const { return mData [inIndex] ; }
  public: inline uint32_t identifier (void) const { return mIdentifier ; }
  public: inline uint32_t frameLength (void) const { return mFrameLength ; }
  public: inline uint32_t stuffBitCount (void) const { return mStuffBitCount ; }
  public: inline uint32_t frameCRC (void) const { return mFrameCRC ; }
  public: bool bitAtIndex (const uint32_t inIndex) const ;

//--- Private methods (used during frame generation)
  private: void enterBitComputeCRCAppendStuff (const bool inBit) ;

  private: void enterBitInFrame (const bool inBit) ;

  private: void enterBitInFrameComputeCRC (const bool inBit) ;

   public: static uint8_t lengthForCode (const uint8_t inDataLengthCode) ;

//--- Private properties
  private: uint32_t mBits [30] ;
  private: uint8_t mData [64] ;
  private: const uint32_t mIdentifier ;
  private: uint32_t mFrameCRC ;
  private: uint32_t mFrameLength ;
  private: uint32_t mCRCAccumulator17 ;
  private: uint32_t mCRCAccumulator21 ;
  private: uint8_t mStuffBitCount ;
  private: const uint8_t mDataLengthCode ;
  private: const FrameFormat mFrameFormat ;
  private: const ProtocolType mProtocolType ;
  private: const AckSlot mAckSlot ;

  private: bool mLastBitValue ;
  private: uint8_t mConsecutiveBitCount ;
} ;

//--------------------------------------------------------------------------------------------------

CANFDFrameBitsGenerator::CANFDFrameBitsGenerator (const uint32_t inIdentifier,
                                                  const FrameFormat inFrameFormat,
                                                  const ProtocolType inProtocolType,
                                                  const uint8_t inDataLengthCode,
                                                  const uint8_t inData [64],
                                                  const AckSlot inAckSlot) :
mBits (),
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
    mBits [i] = UINT32_MAX ;
  }
  const uint8_t dataByteCount = CANFDFrameBitsGenerator::lengthForCode (mDataLengthCode) ;
  for (uint8_t i=0 ; i<dataByteCount ; i++) {
    mData [i] = inData [i] ;
  }
//--- Change CRC initial value according to ISO
  switch (mProtocolType) {
  case PROTOCOL_NON_ISO :
    break ;
  case PROTOCOL_ISO :
    mCRCAccumulator17 = 1U << 16 ;
    mCRCAccumulator21 = 1U << 20 ;
    break ;
  }
//--- Enter SOF
  enterBitComputeCRCAppendStuff (false) ;
//--- Enter Identifier
  switch (mFrameFormat) {
  case FrameFormat::extended :
    for (uint8_t idx = 28 ; idx >= 18 ; idx--) { // Identifier
      const bool bit = (mIdentifier & (1 << idx)) != 0 ;
      enterBitComputeCRCAppendStuff (bit) ;
    }
    enterBitComputeCRCAppendStuff (true) ; // SRR
    enterBitComputeCRCAppendStuff (true) ; // IDE
    for (int idx = 17 ; idx >= 0 ; idx--) { // Identifier
      const bool bit = (mIdentifier & (1 << idx)) != 0 ;
      enterBitComputeCRCAppendStuff (bit) ;
    }
    break ;
  case FrameFormat::standard :
    for (int idx = 10 ; idx >= 0 ; idx--) { // Identifier
      const bool bit = (mIdentifier & (1 << idx)) != 0 ;
      enterBitComputeCRCAppendStuff (bit) ;
    }
    enterBitComputeCRCAppendStuff (false) ; // R1
    break ;
  }
//--- Enter DLC
  enterBitComputeCRCAppendStuff (false) ; // IDE
  enterBitComputeCRCAppendStuff (true) ; // FDF
  enterBitComputeCRCAppendStuff (false) ; // R0
  enterBitComputeCRCAppendStuff (false) ; // BRS
  enterBitComputeCRCAppendStuff (false) ; // ESI
  enterBitComputeCRCAppendStuff ((mDataLengthCode & 8) != 0) ;
  enterBitComputeCRCAppendStuff ((mDataLengthCode & 4) != 0) ;
  enterBitComputeCRCAppendStuff ((mDataLengthCode & 2) != 0) ;
  enterBitComputeCRCAppendStuff ((mDataLengthCode & 1) != 0) ;
//--- Enter DATA
  bool lastBit = mLastBitValue ;
  for (uint8_t dataIdx = 0 ; dataIdx < dataByteCount ; dataIdx ++) {
    for (int bitIdx = 7 ; bitIdx >= 0 ; bitIdx--) {
      lastBit = (inData [dataIdx] & (1 << bitIdx)) != 0 ;
      if ((dataIdx == (dataByteCount - 1)) && (bitIdx == 0)) { // Last data bit
        enterBitInFrameComputeCRC (lastBit) ;
      }else{
        enterBitComputeCRCAppendStuff (lastBit) ;
      }
    }
  }
//--- Enter STUFF BIT COUNT
  switch (mProtocolType) {
  case PROTOCOL_NON_ISO :
    break ;
  case PROTOCOL_ISO :
    { enterBitInFrame (!lastBit) ;
      const uint8_t GRAY_CODE_PARITY [8] = {0, 3, 6, 5, 12, 15, 10, 9} ;
      const uint8_t code = GRAY_CODE_PARITY [mStuffBitCount % 8] ;
      enterBitInFrameComputeCRC ((code & 8) != 0) ;
      enterBitInFrameComputeCRC ((code & 4) != 0) ;
      enterBitInFrameComputeCRC ((code & 2) != 0) ;
      lastBit = (code & 1) != 0 ;
      enterBitInFrameComputeCRC (lastBit) ;
    }
    break ;
  }
//--- Enter CRC SEQUENCE
  enterBitInFrame (!lastBit) ;
  const uint32_t frameCRC = (mDataLengthCode > 10) ? mCRCAccumulator21 : mCRCAccumulator17 ;
  mFrameCRC = frameCRC ;
  const int crcFirstBitIndex = (mDataLengthCode > 10) ? 20 : 16 ;
  uint32_t bitCount = 0 ;
  for (int idx = crcFirstBitIndex ; idx >= 0 ; idx--) {
    const bool crc_bit = (frameCRC & (1 << idx)) != 0 ;
    enterBitInFrame (crc_bit) ;
    bitCount += 1 ;
    if (bitCount == 4) {
      bitCount = 0 ;
      enterBitInFrame (!crc_bit) ;
    }
  }
  enterBitInFrame (true) ;
//--- Enter ACK, EOF, INTERMISSION
  switch (mAckSlot) {
  case ACK_SLOT_DOMINANT :
    enterBitInFrame (false) ;
    break ;
  case ACK_SLOT_RECESSIVE :
    enterBitInFrame (true) ;
    break ;
  }
  enterBitInFrame (true) ;
  for (uint8_t i=0 ; i<7 ; i++) {
    enterBitInFrame (true) ;
  }
  for (uint8_t i=0 ; i<3 ; i++) {
    enterBitInFrame (true) ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDFrameBitsGenerator::enterBitInFrameComputeCRC (const bool inBit) {
//--- Enter bit in frame
  enterBitInFrame (inBit) ;
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

void CANFDFrameBitsGenerator::enterBitComputeCRCAppendStuff (const bool inBit) {
//--- Enter bit in frame
  enterBitInFrameComputeCRC (inBit) ;
//--- Add a stuff bit ?
  if (mLastBitValue == inBit) {
    mConsecutiveBitCount += 1 ;
    if (mConsecutiveBitCount == 5) {
      mConsecutiveBitCount = 1 ;
      mStuffBitCount += 1 ;
      mLastBitValue ^= true ;
      enterBitInFrameComputeCRC (mLastBitValue) ;
    }
  }else{
    mLastBitValue = inBit ;
    mConsecutiveBitCount = 1 ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDFrameBitsGenerator::enterBitInFrame (const bool inBit) {
  if (!inBit) {
    const uint32_t idx = mFrameLength / 32 ;
    const uint32_t offset = mFrameLength % 32 ;
    mBits [idx] &= ~ (1U << offset) ;
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

  while (mSerialSimulationData.GetCurrentSampleNumber() < adjusted_largest_sample_requested) {
    CreateCANFrame () ;
  }

  *simulation_channel = &mSerialSimulationData;
  return 1;
}

//--------------------------------------------------------------------------------------------------

void CANMolinaroSimulationDataGenerator::CreateCANFrame () {
  const U32 samples_per_bit = mSimulationSampleRateHz / mSettings->mBitRate;
  const bool inverted = mSettings->inverted () ;
  const SimulatorGeneratedFrameType frameTypes = mSettings->generatedFrameType () ;
  const ProtocolType protocol = PROTOCOL_NON_ISO ;
//--- Select Frame type to generate
  bool canfd_0_16 = false ;
  bool canfd_24_64 = false ;
  bool extended = false ;
  bool remoteFrame = false ;
  switch (frameTypes) {
  case GENERATE_ALL_FRAME_TYPES :
    extended = (random () & 1) != 0 ;
    remoteFrame = (random () & 1) != 0 ;
    canfd_0_16 = (random () & 1) != 0 ;
    canfd_24_64 = (random () & 1) != 0 ;
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
    canfd_0_16 = true ;
    break ;
  case GENERATE_ONLY_CANFD_EXTENDED_0_16 :
    canfd_0_16 = true ;
    extended = true ;
    break ;
  case GENERATE_ONLY_CANFD_BASE_20_64 :
    canfd_24_64 = true ;
    break ;
  case GENERATE_ONLY_CANFD_EXTENDED_20_64 :
    canfd_24_64 = true ;
    extended = true ;
    break ;
  }
//--- Select ACK SLOT level
  AckSlot ack = AckSlot::ACK_SLOT_DOMINANT ;
  switch (mSettings->generatedAckSlot ()) {
  case GENERATE_ACK_DOMINANT :
    break ;
  case GENERATE_ACK_RECESSIVE :
    ack = AckSlot::ACK_SLOT_RECESSIVE ;
    break ;
  GENERATE_ACK_RANDOMLY :
    ack = ((random () & 1) != 0) ? AckSlot::ACK_SLOT_DOMINANT : AckSlot::ACK_SLOT_RECESSIVE ;
    break ;
  }
//--- Let's move forward for 11 recessive bits
  mSerialSimulationData.TransitionIfNeeded (inverted ? BIT_LOW : BIT_HIGH) ;  // Edge for IDLE
  mSerialSimulationData.Advance (samples_per_bit * 11) ;
  mSerialSimulationData.TransitionIfNeeded (inverted ? BIT_HIGH : BIT_LOW) ;  // Edge for SOF bit
//--- Generate CANFD Frame, 0 to 16 bytes
  if (canfd_0_16) {
    uint8_t data [20] ;
    const FrameFormat format = extended ? FrameFormat::extended : FrameFormat::standard ;
    const uint32_t identifier = uint32_t (random ()) & (extended ? 0x1FFFFFFF : 0x7FF) ;
    const uint8_t dataLengthCode = uint8_t (random ()) % 11 ; // 0 ... 10
    for (uint32_t i=0 ; i<CANFDFrameBitsGenerator::lengthForCode (dataLengthCode) ; i++) {
      data [i] = uint8_t (random ()) ;
    }
    const CANFDFrameBitsGenerator frame (identifier, format, protocol, dataLengthCode, data, ack) ;
  //--- Now, send frame
    for (U32 i=0 ; i < frame.frameLength () ; i++) {
      const bool bit = frame.bitAtIndex (i) ^ inverted ;
      mSerialSimulationData.TransitionIfNeeded (bit ? BIT_HIGH : BIT_LOW) ;
      mSerialSimulationData.Advance (samples_per_bit) ;
    }
//--- Generate CANFD Frame, 24 to 64 bytes
  }else if (canfd_24_64) {
    uint8_t data [64] ;
    const FrameFormat format = extended ? FrameFormat::extended : FrameFormat::standard ;
    const uint32_t identifier = uint32_t (random ()) & (extended ? 0x1FFFFFFF : 0x7FF) ;
    const uint8_t dataLengthCode = uint8_t (random ()) % 5 + 11 ; // 11, ..., 15
    for (uint32_t i=0 ; i<CANFDFrameBitsGenerator::lengthForCode (dataLengthCode) ; i++) {
      data [i] = uint8_t (random ()) ;
    }
    const CANFDFrameBitsGenerator frame (identifier, format, protocol, dataLengthCode, data, ack) ;
  //--- Now, send frame
    for (U32 i=0 ; i < frame.frameLength () ; i++) {
      const bool bit = frame.bitAtIndex (i) ^ inverted ;
      mSerialSimulationData.TransitionIfNeeded (bit ? BIT_HIGH : BIT_LOW) ;
      mSerialSimulationData.Advance (samples_per_bit) ;
    }
//--- Generate CAN2.0B Frame
  }else{
    uint8_t data [8] ;
    const FrameFormat format = extended ? FrameFormat::extended : FrameFormat::standard ;
    const FrameType type = remoteFrame ? FrameType::remote : FrameType::data ;
    const uint32_t identifier = uint32_t (random ()) & (extended ? 0x1FFFFFFF : 0x7FF) ;
    const uint8_t dataLength = uint8_t (random ()) % 9 ;
    if (! remoteFrame) {
      for (uint32_t i=0 ; i<dataLength ; i++) {
        data [i] = uint8_t (random ()) ;
      }
    }
    const CANFrameBitsGenerator frame (identifier, format, dataLength, data, type, ack) ;
  //--- Now, send frame
    for (U32 i=0 ; i < frame.frameLength () ; i++) {
      const bool bit = frame.bitAtIndex (i) ^ inverted ;
      mSerialSimulationData.TransitionIfNeeded (bit ? BIT_HIGH : BIT_LOW) ;
      mSerialSimulationData.Advance (samples_per_bit) ;
    }
  }
//--- We need to end recessive
  mSerialSimulationData.TransitionIfNeeded (inverted ? BIT_LOW : BIT_HIGH) ;
}

//--------------------------------------------------------------------------------------------------
