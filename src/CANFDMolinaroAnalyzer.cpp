#include "CANFDMolinaroAnalyzer.h"
#include "CANFDMolinaroAnalyzerSettings.h"
#include <AnalyzerChannelData.h>

#include <string>
#include <sstream>

//----------------------------------------------------------------------------------------
//   CANFDMolinaroAnalyzer
//----------------------------------------------------------------------------------------

CANFDMolinaroAnalyzer::CANFDMolinaroAnalyzer (void) :
Analyzer2 (),
mSettings (new CANFDMolinaroAnalyzerSettings ()),
mSimulationInitialized (false) {
  SetAnalyzerSettings (mSettings.get()) ;
  UseFrameV2 () ;
}

//----------------------------------------------------------------------------------------

CANFDMolinaroAnalyzer::~CANFDMolinaroAnalyzer (void) {
  KillThread();
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::SetupResults (void) {
  mResults.reset (new CANFDMolinaroAnalyzerResults (this, mSettings.get())) ;
  SetAnalyzerResults (mResults.get()) ;
  mResults->AddChannelBubblesWillAppearOn (mSettings->mInputChannel) ;
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::WorkerThread (void) {
  const bool inverted = mSettings->inverted () ;
  mSampleRateHz = GetSampleRate () ;
  AnalyzerChannelData * serial = GetAnalyzerChannelData (mSettings->mInputChannel) ;
//--- Sample settings
  mCurrentSamplesPerBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
//--- Synchronize to recessive level
  if (serial->GetBitState() == (inverted ? BIT_HIGH : BIT_LOW)) {
    serial->AdvanceToNextEdge () ;
  }
//---
  mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
  mUnstuffingActive = false ;
  mPreviousBit = (serial->GetBitState () == BIT_HIGH) ^ inverted ;
//---
  while (1) {
    const bool currentBitValue = (serial->GetBitState () == BIT_HIGH) ^ inverted ;
    const U64 start = serial->GetSampleNumber () ;
    const U64 nextEdge = serial->GetSampleOfNextEdge () ;

    U64 currentCenter = start + mCurrentSamplesPerBit / 2 ;
    while (currentCenter < nextEdge) {
      enterBit (currentBitValue, currentCenter) ;
      currentCenter += mCurrentSamplesPerBit ;
    }
  //---
    mResults->CommitResults () ;
    serial->AdvanceToNextEdge () ;
  }
}

//----------------------------------------------------------------------------------------

bool CANFDMolinaroAnalyzer::NeedsRerun () {
  return false;
}

//----------------------------------------------------------------------------------------

U32 CANFDMolinaroAnalyzer::GenerateSimulationData (U64 minimum_sample_index,
                                                 U32 device_sample_rate,
                                                 SimulationChannelDescriptor** simulation_channels ) {
  if (mSimulationInitialized == false) {
    mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), mSettings.get() );
    mSimulationInitialized = true;
  }
  return mSimulationDataGenerator.GenerateSimulationData (minimum_sample_index,
                                                          device_sample_rate,
                                                          simulation_channels) ;
}

//----------------------------------------------------------------------------------------

U32 CANFDMolinaroAnalyzer::GetMinimumSampleRateHz () {
  const U32 arbitrationBitRate = mSettings->arbitrationBitRate () ;
  const U32 dataBitRate = mSettings->dataBitRate () ;
  const U32 max = (dataBitRate > arbitrationBitRate) ? dataBitRate : arbitrationBitRate ;
  return max * 12 ;
}

//----------------------------------------------------------------------------------------

const char* CANFDMolinaroAnalyzer::GetAnalyzerName () const {
  return "CANFD (Molinaro)";
}

//----------------------------------------------------------------------------------------

const char* GetAnalyzerName () {
  return "CANFD (Molinaro)";
}

//----------------------------------------------------------------------------------------

Analyzer* CreateAnalyzer () {
  return new CANFDMolinaroAnalyzer();
}

//----------------------------------------------------------------------------------------

void DestroyAnalyzer (Analyzer* analyzer) {
  delete analyzer;
}

//----------------------------------------------------------------------------------------
//  CAN FRAME DECODER
//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBit (const bool inBit, U64 & ioBitCenterSampleNumber) {
  if (!mUnstuffingActive) {
    decodeFrameBit (inBit, ioBitCenterSampleNumber) ;
    mPreviousBit = inBit ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (inBit != mPreviousBit)) {
   // Stuff bit - discarded
    addMark (ioBitCenterSampleNumber, AnalyzerResults::X);
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
    mStuffBitCount += 1 ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (mPreviousBit == inBit)) { // Stuff Error
    addMark (ioBitCenterSampleNumber, AnalyzerResults::ErrorX);
    enterInErrorMode (ioBitCenterSampleNumber + mCurrentSamplesPerBit / 2) ;
    mConsecutiveBitCountOfSamePolarity += 1 ;
  }else if (mPreviousBit == inBit) {
    mConsecutiveBitCountOfSamePolarity += 1 ;
    decodeFrameBit (inBit, ioBitCenterSampleNumber) ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }else{
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
    decodeFrameBit (inBit, ioBitCenterSampleNumber) ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }
}

//----------------------------------------------------------------------------------------

static const uint8_t CANFD_LENGTH [16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64} ;

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::decodeFrameBit (const bool inBit, U64 & ioBitCenterSampleNumber) {
  switch (mFrameFieldEngineState) {
  case FrameFieldEngineState::IDLE :
    handle_IDLE_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::IDENTIFIER :
    handle_IDENTIFIER_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CONTROL_EXTENDED :
    handle_CONTROL_EXTENDED_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CONTROL_BASE :
    handle_CONTROL_BASE_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CONTROL_AFTER_R0 :
    handle_CONTROL_AFTER_R0_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::DATA :
    handle_DATA_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::SBC :
    handle_SBC_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRC15 :
    handle_CRC15_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRC17 :
    handle_CRC17_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRC21 :
    handle_CRC21_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::CRCDEL :
    handle_CRCDEL_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::ACK :
    handle_ACK_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::ENDOFFRAME :
    handle_ENDOFFRAME_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::INTERMISSION :
    handle_INTERMISSION_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  case FrameFieldEngineState::DECODER_ERROR :
    handle_DECODER_ERROR_state (inBit, ioBitCenterSampleNumber) ;
    break ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_IDLE_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  if (inBit) {
    addMark (inBitCenterSampleNumber, AnalyzerResults::Stop) ;
  }else{ // SOF
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
    addMark (inBitCenterSampleNumber, AnalyzerResults::Start);
    mFieldBitIndex = 0 ;
    mIdentifier = 0 ;
    mStuffBitCount = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::IDENTIFIER ;
    mCurrentSamplesPerBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
    mStartOfFieldSampleNumber = inBitCenterSampleNumber + mCurrentSamplesPerBit / 2 ;
    mStartOfFrameSampleNumber = inBitCenterSampleNumber ;
    mMarkerTypeForDataAndCRC = AnalyzerResults::Dot ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_IDENTIFIER_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex <= 11) { // Standard identifier
    addMark (inBitCenterSampleNumber, AnalyzerResults::Dot);
    mIdentifier <<= 1 ;
    mIdentifier |= inBit ;
  }else if (mFieldBitIndex == 12) { // RTR or SRR bit
    mFrameType = inBit ? FrameType::remote : FrameType::canData  ;
  }else if (mFieldBitIndex == 13) { // IDE bit
    mFrameFormat = inBit ? FrameFormat::extended : FrameFormat::base ;
    if (!inBit) { // IDE dominant -> base frame
    //--- RTR mark
      addMark (inBitCenterSampleNumber - mCurrentSamplesPerBit,
               inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
    //--- IDE Mark
      addMark (inBitCenterSampleNumber, AnalyzerResults::DownArrow) ;
    //--- Bubble
      addBubble (STANDARD_IDENTIFIER_FIELD_RESULT,
                 mIdentifier,
                 mFrameType == FrameType::canData, // 0 -> remote, 1 -> data
                 inBitCenterSampleNumber - mCurrentSamplesPerBit) ;
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::CONTROL_BASE ;
    }else{ // IDE recessive -> extended frame
    //--- SRR mark
      addMark (inBitCenterSampleNumber - mCurrentSamplesPerBit, inBit ? AnalyzerResults::One : AnalyzerResults::ErrorSquare) ;
    //--- IDE Mark
      addMark (inBitCenterSampleNumber, AnalyzerResults::UpArrow) ;
    }
  }else if (mFieldBitIndex < 32) { // ID17 ... ID0
    addMark (inBitCenterSampleNumber, AnalyzerResults::Dot);
    mIdentifier <<= 1 ;
    mIdentifier |= inBit ;
  }else{ // RTR
    mFrameType = inBit ? FrameType::remote : FrameType::canData ;
    addMark (inBitCenterSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
  //--- Bubble
    addBubble (EXTENDED_IDENTIFIER_FIELD_RESULT,
               mIdentifier,
               mFrameType == FrameType::canData, // 0 -> remote, 1 -> data
               inBitCenterSampleNumber) ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CONTROL_EXTENDED ;
  }
}

//----------------------------------------------------------------------------------------


void CANFDMolinaroAnalyzer::handle_CONTROL_BASE_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 1) { // FDF bit
    if (inBit) { // FDF recessive -> CANFD frame
      addMark (inBitCenterSampleNumber, AnalyzerResults::UpArrow) ;
      mFrameType = FrameType::canfdData ;
    }else{
      addMark (inBitCenterSampleNumber, AnalyzerResults::DownArrow) ;
      mFieldBitIndex = 0 ;
      mDataCodeLength = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::CONTROL_AFTER_R0 ;
    }
  }else if (inBit) { // R0 bit recessive -> error
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorDot) ;
    enterInErrorMode (inBitCenterSampleNumber) ;
  }else{ // R0 dominant: ok
    addMark (inBitCenterSampleNumber, AnalyzerResults::Zero) ;
    mDataCodeLength = 0 ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CONTROL_AFTER_R0 ;
  }
}

//----------------------------------------------------------------------------------------


void CANFDMolinaroAnalyzer::handle_CONTROL_EXTENDED_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 1) { // FDF bit
    if (inBit) { // FDF recessive -> CANFD frame
      addMark (inBitCenterSampleNumber, AnalyzerResults::UpArrow) ;
      mFrameType = FrameType::canfdData ;
    }else{
      addMark (inBitCenterSampleNumber, AnalyzerResults::DownArrow) ;
    }
  }else if (inBit) { // R0 bit recessive -> error
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorDot) ;
    enterInErrorMode (inBitCenterSampleNumber) ;
  }else{ // R0 dominant: ok
    addMark (inBitCenterSampleNumber, AnalyzerResults::Zero) ;
    mDataCodeLength = 0 ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CONTROL_AFTER_R0 ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CONTROL_AFTER_R0_state (const bool inBit,
                                                           U64 & ioBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  mFieldBitIndex ++ ;
  if (mFrameType == FrameType::canfdData) {
    if (mFieldBitIndex == 1) { // BRS
      mBRS = inBit ;
      if (inBit) { // Switch to data bit rate
        const U64 samplesForDataBitRate = mSampleRateHz / mSettings->dataBitRate () ;
        const U64 BSRsamplesX100 =
          mSettings->arbitrationSamplePoint () * mCurrentSamplesPerBit
        +
          (100 - mSettings->dataSamplePoint ()) * samplesForDataBitRate
        ;
        const U64 centerBSR = ioBitCenterSampleNumber - mCurrentSamplesPerBit / 2 + BSRsamplesX100 / 200 ;
        addMark (centerBSR, AnalyzerResults::UpArrow) ;
      //--- Adjust for center of next bit
        ioBitCenterSampleNumber -= mCurrentSamplesPerBit / 2 ; // Returns at the beginning of BRS bit
        ioBitCenterSampleNumber += BSRsamplesX100 / 100 ; // Advance at the beginning of next bit
        ioBitCenterSampleNumber -= samplesForDataBitRate / 2 ; // Back half of a data bit rate bit
      //--- Switch to Data Bit Rate
        mCurrentSamplesPerBit = samplesForDataBitRate ;
        mMarkerTypeForDataAndCRC = AnalyzerResults::Square ;
      }else{
        addMark (ioBitCenterSampleNumber, AnalyzerResults::DownArrow) ;
      }
    }else if (mFieldBitIndex == 2) { // ESI
      addMark (ioBitCenterSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
      mESI = inBit ;
    }else{
      addMark (ioBitCenterSampleNumber, mMarkerTypeForDataAndCRC) ;
      mDataCodeLength <<= 1 ;
      mDataCodeLength |= inBit ;
      if (mFieldBitIndex == 6) {
        const U32 data2 = U32 (mBRS) | (U32 (mESI) << 1) ;
        addBubble (CANFD_CONTROL_FIELD_RESULT, mDataCodeLength, data2, ioBitCenterSampleNumber) ;
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
    addMark (ioBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
    mDataCodeLength <<= 1 ;
    mDataCodeLength |= inBit ;
    if (mFieldBitIndex == 4) {
      addBubble (CAN20B_CONTROL_FIELD_RESULT, mDataCodeLength, 0, ioBitCenterSampleNumber) ;
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

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_DATA_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  addMark (inBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
  mData [mFieldBitIndex / 8] <<= 1 ;
  mData [mFieldBitIndex / 8] |= inBit ;
  mFieldBitIndex += 1 ;
  if ((mFieldBitIndex % 8) == 0) {
    const U32 dataIndex = (mFieldBitIndex - 1) / 8 ;
    addBubble (DATA_FIELD_RESULT, mData [dataIndex], dataIndex, inBitCenterSampleNumber) ;
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

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRC15_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  enterBitInCRC15 (inBit) ;
  addMark (inBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
  mFieldBitIndex += 1 ;
  if (mFieldBitIndex == 15) {
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
    addBubble (CRC15_FIELD_RESULT, mCRC15, mCRC15Accumulator, inBitCenterSampleNumber) ;
    if (mCRC15Accumulator != 0) {
      mFrameFieldEngineState = DECODER_ERROR ;
    }
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_SBC_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  mFieldBitIndex += 1 ;
  if (mFieldBitIndex == 1) { // Forced Stuff Bit
    mSBCField = 0 ;
    if (inBit == mPreviousBit) {
      addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
      enterInErrorMode (inBitCenterSampleNumber) ;
    }else{
      addMark (inBitCenterSampleNumber, AnalyzerResults::X);
    }
  }else if (mFieldBitIndex <= 4) {
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
    mSBCField <<= 1 ;
    mSBCField |= inBit ;
    addMark (inBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
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
    addMark (inBitCenterSampleNumber, oneBitCountIsEven ? AnalyzerResults::Dot : AnalyzerResults::ErrorX) ;
    const U32 data2 = ((mStuffBitCount % 8) << 1) | !oneBitCountIsEven ;
    addBubble (SBC_FIELD_RESULT, suffBitCountMod8, data2, inBitCenterSampleNumber) ;
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

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRC17_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  if ((mFieldBitIndex % 5) != 0) {
    enterBitInCRC17 (inBit) ;
    addMark (inBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
  }else if (inBit == mPreviousBit) {
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
    enterInErrorMode (inBitCenterSampleNumber) ;
  }else{
    addMark (inBitCenterSampleNumber, AnalyzerResults::X);
  }
  mFieldBitIndex += 1 ;
  if (mFieldBitIndex == 22) {
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
    addBubble (CRC17_FIELD_RESULT, mCRC17, mCRC17Accumulator, inBitCenterSampleNumber) ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRC21_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  if ((mFieldBitIndex % 5) != 0) {
    enterBitInCRC21 (inBit) ;
    addMark (inBitCenterSampleNumber, mMarkerTypeForDataAndCRC);
  }else if (inBit == mPreviousBit) {
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
    enterInErrorMode (inBitCenterSampleNumber) ;
  }else{
    addMark (inBitCenterSampleNumber, AnalyzerResults::X);
  }
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 27) {
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
    addBubble (CRC21_FIELD_RESULT, mCRC21, mCRC21Accumulator, inBitCenterSampleNumber) ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_CRCDEL_state (const bool inBit, U64 & ioBitCenterSampleNumber) {
  mUnstuffingActive = false ;
  if (inBit) { // Handle Bit Rate Switch: data bit rate -> arbitration bit rate
    const U32 samplesPerArbitrationBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
    const U64 CRCDELsamplesX100 =
      mSettings->dataSamplePoint () * mCurrentSamplesPerBit
    +
      (100 - mSettings->arbitrationSamplePoint ()) * samplesPerArbitrationBit
    ;
    const U64 centerCRCDEL = ioBitCenterSampleNumber - mCurrentSamplesPerBit / 2 + CRCDELsamplesX100 / 200 ;
    addMark (centerCRCDEL, AnalyzerResults::One) ;
  //--- Adjust for center of next bit
    ioBitCenterSampleNumber -= mCurrentSamplesPerBit / 2 ; // Returns at the beginning of CRCDEL bit
    ioBitCenterSampleNumber += CRCDELsamplesX100 / 100 ; // Advance at the beginning of next bit
    ioBitCenterSampleNumber -= samplesPerArbitrationBit / 2 ; // Back half of a arbitration bit rate bit
  //--- Switch to Data Bit Rate
    mCurrentSamplesPerBit = samplesPerArbitrationBit ;
  }else{
    addMark (ioBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
    enterInErrorMode (ioBitCenterSampleNumber) ;
  }
  mStartOfFieldSampleNumber = ioBitCenterSampleNumber + mCurrentSamplesPerBit / 2 ;
  mFrameFieldEngineState = FrameFieldEngineState::ACK ;
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_ACK_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  mFieldBitIndex ++ ;
  U8 u8Acked = 0;
  if (mFieldBitIndex == 1) { // ACK SLOT
    addMark (inBitCenterSampleNumber, inBit ? AnalyzerResults::ErrorSquare : AnalyzerResults::DownArrow);
    mAcked = inBit ;
  }else{ // ACK DELIMITER
    addBubble (ACK_FIELD_RESULT, mAcked, 0, inBitCenterSampleNumber) ;
    mFrameFieldEngineState = FrameFieldEngineState::ENDOFFRAME ;
    if (inBit) {
      addMark (inBitCenterSampleNumber, AnalyzerResults::One) ;
    }else{
      addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorDot) ;
      enterInErrorMode (inBitCenterSampleNumber) ;
    }
    mFieldBitIndex = 0 ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_ENDOFFRAME_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  if (inBit) {
    addMark (inBitCenterSampleNumber, AnalyzerResults::One) ;
  }else{
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
    enterInErrorMode (inBitCenterSampleNumber) ;
  }
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 7) {
    addBubble (EOF_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::INTERMISSION ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_INTERMISSION_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  if (inBit) {
    addMark (inBitCenterSampleNumber, AnalyzerResults::One) ;
  }else{
    addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorX) ;
    enterInErrorMode (inBitCenterSampleNumber) ;
  }
  mFieldBitIndex ++ ;
  if (mFieldBitIndex == 3) {
    addBubble (INTERMISSION_FIELD_RESULT, 0, 0, inBitCenterSampleNumber) ;
    mFieldBitIndex = 0 ;
    mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::handle_DECODER_ERROR_state (const bool inBit, const U64 inBitCenterSampleNumber) {
  mUnstuffingActive = false ;
  addMark (inBitCenterSampleNumber, AnalyzerResults::ErrorDot);
  if (mPreviousBit != inBit) {
    mConsecutiveBitCountOfSamePolarity = 1 ;
    mPreviousBit = inBit ;
  }else if (inBit) {
    mConsecutiveBitCountOfSamePolarity += 1 ;
    if (mConsecutiveBitCountOfSamePolarity == 11) {
      addBubble (CAN_ERROR_RESULT, 0, 0, inBitCenterSampleNumber) ;
      mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
    }
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBitInCRC15 (const bool inBit) {
  const bool bit14 = (mCRC15Accumulator & (1 << 14)) != 0 ;
  const bool crc_nxt = inBit ^ bit14 ;
  mCRC15Accumulator <<= 1 ;
  mCRC15Accumulator &= 0x7FFF ;
  if (crc_nxt) {
    mCRC15Accumulator ^= 0x4599 ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBitInCRC17 (const bool inBit) {
  const bool bit16 = (mCRC17Accumulator & (1 << 16)) != 0 ;
  const bool crc_nxt = inBit ^ bit16 ;
  mCRC17Accumulator <<= 1 ;
  mCRC17Accumulator &= 0x1FFFF ;
  if (crc_nxt) {
    mCRC17Accumulator ^= 0x1685B ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterBitInCRC21 (const bool inBit) {
  const bool bit20 = (mCRC21Accumulator & (1 << 20)) != 0 ;
  const bool crc_nxt = inBit ^ bit20 ;
  mCRC21Accumulator <<= 1 ;
  mCRC21Accumulator &= 0x1FFFFF ;
  if (crc_nxt) {
    mCRC21Accumulator ^= 0x102899 ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::addMark (const U64 inBitCenterSampleNumber,
                                     const AnalyzerResults::MarkerType inMarker) {
  mResults->AddMarker (inBitCenterSampleNumber, inMarker, mSettings->mInputChannel);
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::addBubble (const U8 inBubbleType,
                                       const U64 inData1,
                                       const U64 inData2,
                                       const U64 inBitCenterSampleNumber) {
  Frame frame ;
  frame.mType = inBubbleType ;
  frame.mFlags = 0 ;
  frame.mData1 = inData1 ;
  frame.mData2 = inData2 ;
  frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
  const U64 endSampleNumber = inBitCenterSampleNumber + mCurrentSamplesPerBit / 2 ;
  frame.mEndingSampleInclusive = endSampleNumber ;
  mResults->AddFrame (frame) ;

  FrameV2 frameV2 ;
  switch (inBubbleType) {
  case STANDARD_IDENTIFIER_FIELD_RESULT :
    { const U8 idf [2] = { U8 (inData1 >> 8), U8 (inData1) } ;
      frameV2.AddByteArray ("Value", idf, 2) ;
      mResults->AddFrameV2 (frameV2, "Std Idf", mStartOfFieldSampleNumber, endSampleNumber) ;
    }
    break ;
  case EXTENDED_IDENTIFIER_FIELD_RESULT :
    { const U8 idf [4] = {
        U8 (inData1 >> 24), U8 (inData1 >> 16), U8 (inData1 >> 8), U8 (inData1)
      } ;
      frameV2.AddByteArray ("Value", idf, 4) ;
      mResults->AddFrameV2 (frameV2, "Ext Idf", mStartOfFieldSampleNumber, endSampleNumber) ;
    }
    break ;
  case CAN20B_CONTROL_FIELD_RESULT :
    frameV2.AddByte ("Value", inData1) ;
    mResults->AddFrameV2 (frameV2, "Ctrl", mStartOfFieldSampleNumber, endSampleNumber) ;
    break ;
  case CANFD_CONTROL_FIELD_RESULT :
    { frameV2.AddByte ("Value", inData1) ;
      std::stringstream str ;
      str << "Ctrl (FDF" ;
      if ((inData2 & 1) != 0) {
        str << ", BRS" ;
      }
      if ((inData2 & 2) != 0) {
        str << ", ESI" ;
      }
      str << ")" ;
      mResults->AddFrameV2 (frameV2, str.str ().c_str (), mStartOfFieldSampleNumber, endSampleNumber) ;
    }
    break ;
  case DATA_FIELD_RESULT :
    { frameV2.AddByte ("Value", inData1) ;
      std::stringstream str ;
      str << "D" << inData2 ;
      mResults->AddFrameV2 (frameV2, str.str ().c_str (), mStartOfFieldSampleNumber, endSampleNumber) ;
    }
    break ;
  case CRC15_FIELD_RESULT :
    { const U8 crc [2] = { U8 (inData1 >> 8), U8 (inData1) } ;
      frameV2.AddByteArray ("Value", crc, 2) ;
      mResults->AddFrameV2 (frameV2, "CRC15", mStartOfFieldSampleNumber, endSampleNumber) ;
    }
    break ;
  case CRC17_FIELD_RESULT :
    { const U8 crc [3] = { U8 (inData1 >> 16), U8 (inData1 >> 8), U8 (inData1) } ;
      frameV2.AddByteArray ("Value", crc, 3) ;
      mResults->AddFrameV2 (frameV2, "CRC17", mStartOfFieldSampleNumber, endSampleNumber) ;
    }
    break ;
  case CRC21_FIELD_RESULT :
    { const U8 crc [3] = { U8 (inData1 >> 16), U8 (inData1 >> 8), U8 (inData1) } ;
      frameV2.AddByteArray ("Value", crc, 3) ;
      mResults->AddFrameV2 (frameV2, "CRC21", mStartOfFieldSampleNumber, endSampleNumber) ;
    }
    break ;
  case ACK_FIELD_RESULT :
    mResults->AddFrameV2 (frameV2, "ACK", mStartOfFieldSampleNumber, endSampleNumber) ;
    break ;
  case EOF_FIELD_RESULT :
    mResults->AddFrameV2 (frameV2, "EOF", mStartOfFieldSampleNumber, endSampleNumber) ;
    break ;
  case INTERMISSION_FIELD_RESULT :
    mResults->AddFrameV2 (frameV2, "IFS", mStartOfFieldSampleNumber, endSampleNumber) ;
    break ;
  case CAN_ERROR_RESULT :
    mResults->AddFrameV2 (frameV2, "Error", mStartOfFieldSampleNumber, endSampleNumber) ;
    break ;
  }

  mResults->CommitResults () ;
  ReportProgress (frame.mEndingSampleInclusive) ;
//--- Prepare for next bubble
  mStartOfFieldSampleNumber = endSampleNumber ;
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterInErrorMode (const U64 inBitCenterSampleNumber) {
  mStartOfFieldSampleNumber = inBitCenterSampleNumber ;
  mCurrentSamplesPerBit = mSampleRateHz / mSettings->arbitrationBitRate () ;
  mFrameFieldEngineState = DECODER_ERROR ;
  mUnstuffingActive = false ;
}

//----------------------------------------------------------------------------------------
