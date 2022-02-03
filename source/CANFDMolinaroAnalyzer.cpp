#include "CANFDMolinaroAnalyzer.h"
#include "CANFDMolinaroAnalyzerSettings.h"
#include <AnalyzerChannelData.h>

//--------------------------------------------------------------------------------------------------
//   CANFDMolinaroAnalyzer
//--------------------------------------------------------------------------------------------------

CANFDMolinaroAnalyzer::CANFDMolinaroAnalyzer () :
Analyzer2 (),
mSettings (new CANFDMolinaroAnalyzerSettings ()),
mSimulationInitilized (false),
mDataBitRateActive (false) {
  SetAnalyzerSettings (mSettings.get()) ;
}

//--------------------------------------------------------------------------------------------------

CANFDMolinaroAnalyzer::~CANFDMolinaroAnalyzer () {
  KillThread();
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::SetupResults () {
  mResults.reset (new CANFDMolinaroAnalyzerResults (this, mSettings.get())) ;
  SetAnalyzerResults (mResults.get()) ;
  mResults->AddChannelBubblesWillAppearOn (mSettings->mInputChannel) ;
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::WorkerThread () {
  const bool inverted = mSettings->inverted () ;
  mSampleRateHz = GetSampleRate () ;
  mSerial = GetAnalyzerChannelData (mSettings->mInputChannel) ;
//--- Sample settings
  const U32 samplesPerArbitrationBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
  const U32 samplesPerDataBit = mSampleRateHz / mSettings->dataBitRate () ;
//--- Synchronize to recessive level
  if (mSerial->GetBitState() == (inverted ? BIT_HIGH : BIT_LOW)) {
    mSerial->AdvanceToNextEdge () ;
  }
  while (1) {
  //--- Synchronize on falling edge: this SOF bit
    mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
    mUnstuffingActive = false ;
    mDataBitRateActive = false ;
  //--- Loop util the end of the frame (11 consecutive high bits)
    bool currentBitState = true ;
    do{
      mSerial->AdvanceToNextEdge () ;
      currentBitState ^= true ;
      const U64 start = mSerial->GetSampleNumber () ;
      const U64 nextEdge = mSerial->GetSampleOfNextEdge () ;
      const U64 bitCount = (nextEdge - start + samplesPerArbitrationBit / 2) / samplesPerArbitrationBit ;
      for (U64 i=0 ; i<bitCount ; i++) {
        enterBit (currentBitState, start + samplesPerArbitrationBit / 2 + i * samplesPerArbitrationBit) ;
      }

//     do{
//       mSerial->AdvanceToNextEdge () ;
//       currentBitState ^= true ;
//       const U64 start = mSerial->GetSampleNumber () ;
//       const U64 nextEdge = mSerial->GetSampleOfNextEdge () ;
//       U64 currentSample = start ;
//       U64 samplesPerBit = mDataBitRateActive ? samplesPerDataBit : samplesPerArbitrationBit ;
//       while ((currentSample + samplesPerBit / 2) < nextEdge) {
//         currentSample += samplesPerBit / 2 ; // Advance to center of bit
//         enterBit (currentBitState, currentSample) ;
//         samplesPerBit = mDataBitRateActive ? samplesPerDataBit : samplesPerArbitrationBit ;
//         currentSample += samplesPerBit / 2 ; // Advance to start of next bit
//       }


//       const U64 bitCount = (nextEdge - start + samplesPerArbitrationBit / 2) / samplesPerArbitrationBit ;
//       for (U64 i=0 ; i<bitCount ; i++) {
//         enterBit (currentBitState, start + samplesPerArbitrationBit / 2 + i * samplesPerArbitrationBit) ;
//       }
    }while (mFrameFieldEngineState != FrameFieldEngineState::IDLE) ;
  //---
    mResults->CommitResults () ;
  }
}

//--------------------------------------------------------------------------------------------------

bool CANFDMolinaroAnalyzer::NeedsRerun () {
  return false;
}

//--------------------------------------------------------------------------------------------------

U32 CANFDMolinaroAnalyzer::GenerateSimulationData (U64 minimum_sample_index,
                                                 U32 device_sample_rate,
                                                 SimulationChannelDescriptor** simulation_channels ) {
  if( mSimulationInitilized == false ) {
    mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), mSettings.get() );
    mSimulationInitilized = true;
  }
  return mSimulationDataGenerator.GenerateSimulationData (minimum_sample_index,
                                                          device_sample_rate,
                                                          simulation_channels) ;
}

//--------------------------------------------------------------------------------------------------

U32 CANFDMolinaroAnalyzer::GetMinimumSampleRateHz () {
  const U32 arbitrationBitRate = mSettings->arbitrationBitRate () ;
  const U32 dataBitRate = mSettings->dataBitRate () ;
  const U32 max = (dataBitRate > arbitrationBitRate) ? dataBitRate : arbitrationBitRate ;
  return max * 12 ;
}

//--------------------------------------------------------------------------------------------------

const char* CANFDMolinaroAnalyzer::GetAnalyzerName () const {
  return "CANFD (Molinaro)";
}

//--------------------------------------------------------------------------------------------------

const char* GetAnalyzerName () {
  return "CANFD (Molinaro)";
}

//--------------------------------------------------------------------------------------------------

Analyzer* CreateAnalyzer () {
  return new CANFDMolinaroAnalyzer();
}

//--------------------------------------------------------------------------------------------------

void DestroyAnalyzer (Analyzer* analyzer) {
  delete analyzer;
}

//--------------------------------------------------------------------------------------------------
//  CAN FRAME DECODER
//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBit (const bool inBit, const U64 inSampleNumber) {
  if (!mUnstuffingActive) {
    decodeFrameBit (inBit, inSampleNumber) ;
    mPreviousBit = inBit ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (inBit != mPreviousBit)) {
   // Stuff bit - discarded
    addMark (inSampleNumber, AnalyzerResults::X);
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
    mStuffBitCount += 1 ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (mPreviousBit == inBit)) { // Stuff Error
    addMark (inSampleNumber, AnalyzerResults::ErrorX);
    const U32 samplesPerBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
    enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
    mConsecutiveBitCountOfSamePolarity += 1 ;
  }else if (mPreviousBit == inBit) {
    mConsecutiveBitCountOfSamePolarity += 1 ;
    decodeFrameBit (inBit, inSampleNumber) ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }else{
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
    decodeFrameBit (inBit, inSampleNumber) ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }
}

//--------------------------------------------------------------------------------------------------

static const uint8_t CANFD_LENGTH [16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64} ;

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::decodeFrameBit (const bool inBit, const U64 inSampleNumber) {
  switch (mFrameFieldEngineState) {
  case FrameFieldEngineState::IDLE :
    handle_IDLE_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::IDENTIFIER :
    handle_IDENTIFIER_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::CONTROL_EXTENDED :
    handle_CONTROL_EXTENDED_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::CONTROL_BASE :
    handle_CONTROL_BASE_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::CONTROL_AFTER_R0 :
    handle_CONTROL_AFTER_R0_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::DATA :
    handle_DATA_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::SBC :
    handle_SBC_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRC15 :
    handle_CRC15_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRC17 :
    handle_CRC17_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRC21 :
    handle_CRC21_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRCDEL :
    handle_CRCDEL_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::ACK :
    handle_ACK_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::ENDOFFRAME :
    handle_ENDOFFRAME_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::INTERMISSION :
    handle_INTERMISSION_state (inBit, inSampleNumber) ;
    break ;
  case FrameFieldEngineState::DECODER_ERROR :
    handle_DECODER_ERROR_state (inBit, inSampleNumber) ;
    break ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_IDLE_state (const bool inBit, const U64 inSampleNumber) {
  if (!inBit) {
    mUnstuffingActive = true ;
    mCRC15Accumulator = 0 ;
    switch (mSettings->protocol ()) {
    case CANFD_NON_ISO_PROTOCOL :
      mCRC17Accumulator = 0 ;
      mCRC21Accumulator = 0 ;
      break ;
    case CANFD_ISO_PROTOCOL :
      mCRC17Accumulator = 1 << 16 ;
      mCRC21Accumulator = 1 << 20 ;
      break ;
    }
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = false ;
    enterBitInCRC15 (inBit) ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
    addMark (inSampleNumber, AnalyzerResults::Start);
    mFieldBitIndex = 0 ;
    mIdentifier = 0 ;
    mStuffBitCount = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::IDENTIFIER ;
    const U32 samplesPerArbitrationBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
    mStartOfFieldSampleNumber = inSampleNumber + samplesPerArbitrationBit / 2 ;
    mStartOfFrameSampleNumber = inSampleNumber ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_IDENTIFIER_state (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerArbitrationBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex <= 11) { // Standard identifier
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mIdentifier <<= 1 ;
    mIdentifier |= inBit ;
  }else if (mFieldBitIndex == 12) { // RTR or SRR bit
    mFrameType = inBit ? FrameType::remote : FrameType::canData  ;
  }else if (mFieldBitIndex == 13) { // IDE bit
    mFrameFormat = inBit ? FrameFormat::extended : FrameFormat::base ;
    if (!inBit) { // IDE dominant -> base frame
    //--- RTR mark
      addMark (inSampleNumber - samplesPerArbitrationBit,
               inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
    //--- IDE Mark
      addMark (inSampleNumber, AnalyzerResults::DownArrow) ;
    //--- Bubble
      addBubble (STANDARD_IDENTIFIER_FIELD_RESULT,
                 mIdentifier,
                 mFrameType == FrameType::canData, // 0 -> remote, 1 -> data
                 inSampleNumber - samplesPerArbitrationBit) ;
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::CONTROL_BASE ;
    }else{ // IDE recessive -> extended frame
    //--- SRR mark
      addMark (inSampleNumber - samplesPerArbitrationBit, inBit ? AnalyzerResults::One : AnalyzerResults::ErrorSquare) ;
    //--- IDE Mark
      addMark (inSampleNumber, AnalyzerResults::UpArrow) ;
    }
  }else if (mFieldBitIndex < 32) { // ID17 ... ID0
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mIdentifier <<= 1 ;
    mIdentifier |= inBit ;
  }else{ // RTR
    mFrameType = inBit ? FrameType::remote : FrameType::canData ;
    addMark (inSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
  //--- Bubble
    addBubble (EXTENDED_IDENTIFIER_FIELD_RESULT,
               mIdentifier,
               mFrameType == FrameType::canData, // 0 -> remote, 1 -> data
               inSampleNumber) ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CONTROL_EXTENDED ;
  }
}

//--------------------------------------------------------------------------------------------------


void CANFDMolinaroAnalyzer::handle_CONTROL_BASE_state (const bool inBit, const U64 inSampleNumber) {
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 1) { // FDF bit
    if (inBit) { // FDF recessive -> CANFD frame
      addMark (inSampleNumber, AnalyzerResults::UpArrow) ;
      mFrameType = FrameType::canfdData ;
    }else{
      addMark (inSampleNumber, AnalyzerResults::DownArrow) ;
      mFieldBitIndex = 0 ;
      mDataCodeLength = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::CONTROL_AFTER_R0 ;
    }
  }else if (inBit) { // R0 bit recessive -> error
    enterInErrorMode (inSampleNumber) ;
  }else{ // R0 dominant: ok
    addMark (inSampleNumber, AnalyzerResults::Zero) ;
    mDataCodeLength = 0 ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CONTROL_AFTER_R0 ;
  }
}

//--------------------------------------------------------------------------------------------------


void CANFDMolinaroAnalyzer::handle_CONTROL_EXTENDED_state (const bool inBit, const U64 inSampleNumber) {
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 1) { // FDF bit
    if (inBit) { // FDF recessive -> CANFD frame
      addMark (inSampleNumber, AnalyzerResults::UpArrow) ;
      mFrameType = FrameType::canfdData ;
    }else{
      addMark (inSampleNumber, AnalyzerResults::DownArrow) ;
    }
  }else if (inBit) { // R0 bit recessive -> error
    enterInErrorMode (inSampleNumber) ;
  }else{ // R0 dominant: ok
    addMark (inSampleNumber, AnalyzerResults::Zero) ;
    mDataCodeLength = 0 ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CONTROL_AFTER_R0 ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CONTROL_AFTER_R0_state (const bool inBit, const U64 inSampleNumber) {
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFrameType == FrameType::canfdData) {
    if (mFieldBitIndex == 1) { // BRS
      addMark (inSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
      mBRS = inBit ;
      mDataBitRateActive = inBit ;
    }else if (mFieldBitIndex == 2) { // ESI
      addMark (inSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
      mESI = inBit ;
    }else{
      addMark (inSampleNumber, AnalyzerResults::Dot);
      mDataCodeLength <<= 1 ;
      mDataCodeLength |= inBit ;
      if (mFieldBitIndex == 6) {
        const U32 data2 = U32 (mBRS) | (U32 (mESI) << 1) ;
        addBubble (CANFD_CONTROL_FIELD_RESULT, mDataCodeLength, data2, inSampleNumber) ;
        mFieldBitIndex = 0 ;
        if (mDataCodeLength != 0) {
          mFrameFieldEngineState = FrameFieldEngineState::DATA ;
        }else if (mSettings->protocol () == CANFD_NON_ISO_PROTOCOL) { // No Data, CANFD non ISO
          mCRC17 = mCRC17Accumulator ;
          mUnstuffingActive = false ;
          mFrameFieldEngineState = FrameFieldEngineState::CRC17 ;
        }else{  // No Data, CANFD ISO
          mUnstuffingActive = false ;
          mFrameFieldEngineState = FrameFieldEngineState::SBC ;
        }
      }
    }
  }else{ // Base frame
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mDataCodeLength <<= 1 ;
    mDataCodeLength |= inBit ;
    if (mFieldBitIndex == 4) {
      addBubble (CAN20B_CONTROL_FIELD_RESULT, mDataCodeLength, 0, inSampleNumber) ;
      mFieldBitIndex = 0 ;
      if ((mDataCodeLength > 8) && (mFrameType != FrameType::canfdData)) {
        mDataCodeLength = 8 ;
      }
      mCRC15 = mCRC15Accumulator ;
      if (mFrameType == FrameType::remote) {
        mFrameFieldEngineState = FrameFieldEngineState::CRC15 ;
      }else if (mDataCodeLength > 0) {
        mFrameFieldEngineState = FrameFieldEngineState::DATA ;
      }else if (mFrameType == FrameType::canData) {
        mFrameFieldEngineState = FrameFieldEngineState::CRC15 ;
      }
    }
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_DATA_state (const bool inBit, const U64 inSampleNumber) {
  enterBitInCRC15 (inBit) ;
  addMark (inSampleNumber, AnalyzerResults::Dot);
  mData [mFieldBitIndex / 8] <<= 1 ;
  mData [mFieldBitIndex / 8] |= inBit ;
  mFieldBitIndex += 1 ;
  if ((mFieldBitIndex % 8) == 0) {
    const U32 dataIndex = (mFieldBitIndex - 1) / 8 ;
    addBubble (DATA_FIELD_RESULT, mData [dataIndex], dataIndex, inSampleNumber) ;
  }
  if (mFieldBitIndex == (8 * CANFD_LENGTH [mDataCodeLength])) {
    mFieldBitIndex = 0 ;
    if (mFrameType != FrameType::canfdData) {
      mCRC15 = mCRC15Accumulator ;
      mFrameFieldEngineState = FrameFieldEngineState::CRC15 ;
    }else if (mSettings->protocol () == CANFD_ISO_PROTOCOL) {
      mFrameFieldEngineState = FrameFieldEngineState::SBC ;
      mUnstuffingActive = false ;
    }else if (mDataCodeLength <= 10) {
      mCRC17 = mCRC17Accumulator ;
      mFrameFieldEngineState = FrameFieldEngineState::CRC17 ;
      mUnstuffingActive = false ;
    }else{
      mCRC21 = mCRC21Accumulator ;
      mFrameFieldEngineState = FrameFieldEngineState::CRC21 ;
      mUnstuffingActive = false ;
    }
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRC15_state (const bool inBit, const U64 inSampleNumber) {
  enterBitInCRC15 (inBit) ;
  addMark (inSampleNumber, AnalyzerResults::Dot);
  mFieldBitIndex += 1 ;
  if (mFieldBitIndex == 15) {
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
    addBubble (CRC15_FIELD_RESULT, mCRC15, mCRC15Accumulator, inSampleNumber) ;
    if (mCRC15Accumulator != 0) {
      mFrameFieldEngineState = DECODER_ERROR ;
    }
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_SBC_state (const bool inBit, const U64 inSampleNumber) {
  mFieldBitIndex += 1 ;
  if (mFieldBitIndex == 1) { // Forced Stuff Bit
    mSBCField = 0 ;
    if (inBit == mPreviousBit) {
      enterInErrorMode (inSampleNumber) ;
    }else{
      addMark (inSampleNumber, AnalyzerResults::X);
    }
  }else if (mFieldBitIndex <= 4) {
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
    mSBCField <<= 1 ;
    mSBCField |= inBit ;
    addMark (inSampleNumber, AnalyzerResults::Dot);
  }else{ // Parity bit
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
    const U8 GRAY_CODE_DECODER [8] = {0, 1, 3, 2, 7, 6, 4, 5} ;
    const U8 suffBitCountMod8 = GRAY_CODE_DECODER [mSBCField] ;
    mSBCField <<= 1 ;
    mSBCField |= inBit ;
  //--- Check parity
    bool oneBitCountIsEven = true ;
    U32 v = mSBCField ;
    while (v > 0) {
      oneBitCountIsEven ^= (v & 1) != 0 ;
      v >>= 1 ;
    }
    addMark (inSampleNumber, oneBitCountIsEven ? AnalyzerResults::Dot : AnalyzerResults::ErrorX) ;
    const U32 data2 = ((mStuffBitCount % 8) << 1) | !oneBitCountIsEven ;
    addBubble (SBC_FIELD_RESULT, suffBitCountMod8, data2, inSampleNumber) ;
    mUnstuffingActive = false ;
    mFieldBitIndex = 0 ;
    if (mDataCodeLength <= 10) {
      mCRC17 = mCRC17Accumulator ;
      mFrameFieldEngineState = FrameFieldEngineState::CRC17 ;
    }else{
      mCRC21 = mCRC21Accumulator ;
      mFrameFieldEngineState = FrameFieldEngineState::CRC21 ;
    }
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRC17_state (const bool inBit, const U64 inSampleNumber) {
  if ((mFieldBitIndex % 5) != 0) {
    enterBitInCRC17 (inBit) ;
    addMark (inSampleNumber, AnalyzerResults::Dot);
  }else if (inBit == mPreviousBit) {
    enterInErrorMode (inSampleNumber) ;
  }else{
    addMark (inSampleNumber, AnalyzerResults::X);
  }
  mFieldBitIndex += 1 ;
  if (mFieldBitIndex == 22) {
    mFieldBitIndex = 0 ;
    mDataBitRateActive = false ;
    mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
    addBubble (CRC17_FIELD_RESULT, mCRC17, mCRC17Accumulator, inSampleNumber) ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRC21_state (const bool inBit, const U64 inSampleNumber) {
  if ((mFieldBitIndex % 5) != 0) {
    enterBitInCRC21 (inBit) ;
    addMark (inSampleNumber, AnalyzerResults::Dot);
  }else if (inBit == mPreviousBit) {
    enterInErrorMode (inSampleNumber) ;
  }else{
    addMark (inSampleNumber, AnalyzerResults::X);
  }
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 27) {
    mFieldBitIndex = 0 ;
    mDataBitRateActive = false ;
    mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
    addBubble (CRC21_FIELD_RESULT, mCRC21, mCRC21Accumulator, inSampleNumber) ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRCDEL_state (const bool inBit, const U64 inSampleNumber) {
  mUnstuffingActive = false ;
  if (inBit) {
    addMark (inSampleNumber, AnalyzerResults::One) ;
  }else{
    enterInErrorMode (inSampleNumber) ;
  }
  const U32 samplesPerArbitrationBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
  mStartOfFieldSampleNumber = inSampleNumber + samplesPerArbitrationBit / 2 ;
  mFrameFieldEngineState = FrameFieldEngineState::ACK ;
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_ACK_state (const bool inBit, const U64 inSampleNumber) {
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 1) { // ACK SLOT
    addMark (inSampleNumber, inBit ? AnalyzerResults::ErrorSquare : AnalyzerResults::Square);
  }else{ // ACK DELIMITER
    addBubble (ACK_FIELD_RESULT, 0, 0, inSampleNumber) ;
    if (inBit) {
      addMark (inSampleNumber, AnalyzerResults::One) ;
    }else{
      enterInErrorMode (inSampleNumber) ;
    }
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::ENDOFFRAME ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_ENDOFFRAME_state (const bool inBit, const U64 inSampleNumber) {
  if (inBit) {
    addMark (inSampleNumber, AnalyzerResults::One) ;
  }else{
    enterInErrorMode (inSampleNumber) ;
  }
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 7) {
    addBubble (EOF_FIELD_RESULT, 0, 0, inSampleNumber) ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::INTERMISSION ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_INTERMISSION_state (const bool inBit, const U64 inSampleNumber) {
  if (inBit) {
    addMark (inSampleNumber, AnalyzerResults::One) ;
  }else{
    enterInErrorMode (inSampleNumber) ;
  }
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 3) {
    addBubble (INTERMISSION_FIELD_RESULT, 0, 0, inSampleNumber) ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_DECODER_ERROR_state (const bool inBit, const U64 inSampleNumber) {
  mUnstuffingActive = false ;
  addMark (inSampleNumber, AnalyzerResults::ErrorDot);
  if (mPreviousBit != inBit) {
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
  }else if (inBit) {
    mConsecutiveBitCountOfSamePolarity += 1 ;
    if (mConsecutiveBitCountOfSamePolarity == 11) {
      addBubble (CAN_ERROR_RESULT, 0, 0, inSampleNumber) ;
      mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
    }
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBitInCRC15 (const bool inBit) {
  const bool bit14 = (mCRC15Accumulator & (1 << 14)) != 0 ;
  const bool crc_nxt = inBit ^ bit14 ;
  mCRC15Accumulator <<= 1 ;
  mCRC15Accumulator &= 0x7FFF ;
  if (crc_nxt) {
    mCRC15Accumulator ^= 0x4599 ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBitInCRC17 (const bool inBit) {
  const bool bit16 = (mCRC17Accumulator & (1 << 16)) != 0 ;
  const bool crc_nxt = inBit ^ bit16 ;
  mCRC17Accumulator <<= 1 ;
  mCRC17Accumulator &= 0x1FFFF ;
  if (crc_nxt) {
    mCRC17Accumulator ^= 0x1685B ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBitInCRC21 (const bool inBit) {
  const bool bit20 = (mCRC21Accumulator & (1 << 20)) != 0 ;
  const bool crc_nxt = inBit ^ bit20 ;
  mCRC21Accumulator <<= 1 ;
  mCRC21Accumulator &= 0x1FFFFF ;
  if (crc_nxt) {
    mCRC21Accumulator ^= 0x102899 ;
  }
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::addMark (const U64 inSampleNumber,
                                     const AnalyzerResults::MarkerType inMarker) {
  mResults->AddMarker (inSampleNumber, inMarker, mSettings->mInputChannel);
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::addBubble (const U8 inBubbleType,
                                       const U64 inData1,
                                       const U64 inData2,
                                       const U64 inSampleNumber) {
  Frame frame ;
  frame.mType = inBubbleType ;
  frame.mFlags = 0 ;
  frame.mData1 = inData1 ;
  frame.mData2 = inData2 ;
  frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
  const U64 endSampleNumber = inSampleNumber + mSampleRateHz / (mSettings->arbitrationBitRate () * 2) ;
  frame.mEndingSampleInclusive = endSampleNumber ;
  mResults->AddFrame (frame) ;
  ReportProgress (frame.mEndingSampleInclusive) ;
//--- Prepare for next bubble
  mStartOfFieldSampleNumber = endSampleNumber ;
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterInErrorMode (const U64 inSampleNumber) {
  addMark (inSampleNumber, AnalyzerResults::ErrorX);
  const U32 samplesPerArbitrationBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
  mStartOfFieldSampleNumber = inSampleNumber + samplesPerArbitrationBit / 2 ;
  mFrameFieldEngineState = FrameFieldEngineState::DECODER_ERROR ;
  mUnstuffingActive = false ;
  mDataBitRateActive = false ;
}

//--------------------------------------------------------------------------------------------------


