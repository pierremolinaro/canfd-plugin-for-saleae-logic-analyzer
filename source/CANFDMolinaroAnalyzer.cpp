#include "CANFDMolinaroAnalyzer.h"
#include "CANFDMolinaroAnalyzerSettings.h"
#include <AnalyzerChannelData.h>

//--------------------------------------------------------------------------------------------------
//   CANFDMolinaroAnalyzer
//--------------------------------------------------------------------------------------------------

CANFDMolinaroAnalyzer::CANFDMolinaroAnalyzer () :
Analyzer2(),
mSettings (new CANFDMolinaroAnalyzerSettings ()),
mSimulationInitilized (false) {
  SetAnalyzerSettings( mSettings.get() );
}

//--------------------------------------------------------------------------------------------------

CANFDMolinaroAnalyzer::~CANFDMolinaroAnalyzer () {
  KillThread();
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::SetupResults () {
  mResults.reset( new CANFDMolinaroAnalyzerResults( this, mSettings.get() ) );
  SetAnalyzerResults( mResults.get() );
  mResults->AddChannelBubblesWillAppearOn( mSettings->mInputChannel );
}


//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::WorkerThread () {
  const bool inverted = mSettings->inverted () ;
  mSampleRateHz = GetSampleRate () ;
  mSerial = GetAnalyzerChannelData (mSettings->mInputChannel) ;
//--- Sample settings
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
//--- Synchronize to recessive level
  if (mSerial->GetBitState() == (inverted ? BIT_HIGH : BIT_LOW)) {
    mSerial->AdvanceToNextEdge () ;
  }
  while (1) {
  //--- Synchronize on falling edge: this SOF bit
    mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
    mUnstuffingActive = false ;
  //--- Loop util the end of the frame (11 consecutive high bits)
    bool currentBitState = true ;
    do{
      mSerial->AdvanceToNextEdge () ;
      currentBitState ^= true ;
      const U64 start = mSerial->GetSampleNumber () ;
      const U64 nextEdge = mSerial->GetSampleOfNextEdge () ;
      const U64 bitCount = (nextEdge - start + samplesPerBit / 2) / samplesPerBit ;
      for (U64 i=0 ; i<bitCount ; i++) {
        enterBit (currentBitState, start + samplesPerBit / 2 + i * samplesPerBit) ;
      }
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
  return mSimulationDataGenerator.GenerateSimulationData( minimum_sample_index, device_sample_rate, simulation_channels );
}

//--------------------------------------------------------------------------------------------------

U32 CANFDMolinaroAnalyzer::GetMinimumSampleRateHz() {
  return mSettings->mBitRate * 5;
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
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }else if ((mConsecutiveBitCountOfSamePolarity == 5) && (mPreviousBit == inBit)) { // Stuff Error
    addMark (inSampleNumber, AnalyzerResults::ErrorX);
    const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
    enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
    mConsecutiveBitCountOfSamePolarity += 1 ;
  }else if (mPreviousBit == inBit) {
    mConsecutiveBitCountOfSamePolarity += 1 ;
    decodeFrameBit (inBit, inSampleNumber) ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }else{
    mConsecutiveBitCountOfSamePolarity = 1 ;
    decodeFrameBit (inBit, inSampleNumber) ;
    mPreviousBit = inBit ;
    enterBitInCRC17 (inBit) ;
    enterBitInCRC21 (inBit) ;
  }
}

//--------------------------------------------------------------------------------------------------

static const uint8_t CANFD_LENGTH [16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64} ;

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::decodeFrameBit (const bool inBit, const U64 inSampleNumber) {
  const U32 samplesPerBit = mSampleRateHz / mSettings->mBitRate ;
  switch (mFrameFieldEngineState) {
  case FrameFieldEngineState::IDLE :
    if (!inBit) {
      mUnstuffingActive = true ;
      mCRC15Accumulator = 0 ;
      mCRC17Accumulator = 0 ;
      mCRC21Accumulator = 0 ;
      mConsecutiveBitCountOfSamePolarity = 1 ;
      mPreviousBit = false ;
      enterBitInCRC15 (inBit) ;
      addMark (inSampleNumber, AnalyzerResults::Start);
      mFieldBitIndex = 0 ;
      mIdentifier = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::IDENTIFIER ;
      mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
    }
    break ;
  case FrameFieldEngineState::IDENTIFIER :
    enterBitInCRC15 (inBit) ;
    mFieldBitIndex ++ ;
    if (mFieldBitIndex <= 11) { // Standard identifier
      addMark (inSampleNumber, AnalyzerResults::Dot);
      mIdentifier <<= 1 ;
      mIdentifier |= inBit ;
    }else if (mFieldBitIndex == 12) { // RTR bit
      addMark (inSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
      mFrameType = inBit ? FrameType::remote : FrameType::canData  ;
    }else{ // IDE
      addMark (inSampleNumber, AnalyzerResults::Dot);
      mFieldBitIndex = 0 ;
      if (inBit) {
        mFrameFieldEngineState = FrameFieldEngineState::EXTENDED_IDF ;
      }else{
        addBubble (STANDARD_IDENTIFIER_FIELD_RESULT,
                   mIdentifier,
                   mFrameType == FrameType::canData, // 0 -> remote, 1 -> data
                   inSampleNumber + samplesPerBit / 2) ;
        mDataCodeLength = 0 ;
        mFrameFieldEngineState = FrameFieldEngineState::CONTROL ;
      }
    }
    break ;
  case FrameFieldEngineState::EXTENDED_IDF :
    enterBitInCRC15 (inBit) ;
    mFieldBitIndex ++ ;
    if (mFieldBitIndex <= 18) { // Extended identifier
      addMark (inSampleNumber, AnalyzerResults::Dot);
      mIdentifier <<= 1 ;
      mIdentifier |= inBit ;
    }else if (mFieldBitIndex == 19) { // RTR bit
      addMark (inSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
      mFrameType = inBit ? FrameType::remote : FrameType::canData  ;
    }else{ // R1: should be dominant
      addBubble (EXTENDED_IDENTIFIER_FIELD_RESULT,
                 mIdentifier,
                 mFrameType == FrameType::canData, // 0 -> remote, 1 -> data
                 inSampleNumber + samplesPerBit / 2) ;
      mDataCodeLength = 0 ;
      mFieldBitIndex = 0 ;
      if (inBit) {
        enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
      }else{
        addMark (inSampleNumber, AnalyzerResults::Zero) ;
        mFrameFieldEngineState = FrameFieldEngineState::CONTROL ;
      }
    }
    break ;
  case FrameFieldEngineState::CONTROL :
    enterBitInCRC15 (inBit) ;
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 1) { // R0, ou FDF
      addMark (inSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
      if (inBit) {
        mFrameType = FrameType::canfdData ;
      }
    }else if ((mFrameType == FrameType::canfdData) && (mFieldBitIndex == 2)) { // R0
      if (inBit) {
        enterInErrorMode (inSampleNumber + samplesPerBit / 2) ;
      }else{
        addMark (inSampleNumber, AnalyzerResults::Zero) ;
      }
    }else if ((mFrameType == FrameType::canfdData) && (mFieldBitIndex == 3)) { // BRS
      addMark (inSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
    }else if ((mFrameType == FrameType::canfdData) && (mFieldBitIndex == 4)) { // ESI
      addMark (inSampleNumber, inBit ? AnalyzerResults::UpArrow : AnalyzerResults::DownArrow) ;
    }else{
      addMark (inSampleNumber, AnalyzerResults::Dot);
      mDataCodeLength <<= 1 ;
      mDataCodeLength |= inBit ;
      if (mFieldBitIndex == ((mFrameType == FrameType::canfdData) ? 8 : 5)) {
        addBubble (CONTROL_FIELD_RESULT,
                   mDataCodeLength,
                   mFrameType == FrameType::canfdData,
                   inSampleNumber + samplesPerBit / 2) ;
        mFieldBitIndex = 0 ;
        if ((mDataCodeLength > 8) && (mFrameType != FrameType::canfdData)) {
          mDataCodeLength = 8 ;
        }
        mCRC15 = mCRC15Accumulator ;
        mCRC17 = mCRC17Accumulator ;
        if (mFrameType == FrameType::remote) {
          mFrameFieldEngineState = FrameFieldEngineState::CRC15 ;
        }else if (mDataCodeLength > 0) {
          mFrameFieldEngineState = FrameFieldEngineState::DATA ;
        }else if (mFrameType == FrameType::canData) {
          mFrameFieldEngineState = FrameFieldEngineState::CRC15 ;
        }else{
          mFrameFieldEngineState = FrameFieldEngineState::CRC17 ;
          mUnstuffingActive = false ;
        }
      }
    }
    break ;
  case FrameFieldEngineState::DATA :
    enterBitInCRC15 (inBit) ;
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mData [mFieldBitIndex / 8] <<= 1 ;
    mData [mFieldBitIndex / 8] |= inBit ;
    mFieldBitIndex ++ ;
    if ((mFieldBitIndex % 8) == 0) {
      const U32 dataIndex = (mFieldBitIndex - 1) / 8 ;
      addBubble (DATA_FIELD_RESULT, mData [dataIndex], dataIndex, inSampleNumber + samplesPerBit / 2) ;
    }
    if (mFieldBitIndex == (8 * CANFD_LENGTH [mDataCodeLength])) {
      if (mFrameType != FrameType::canfdData) {
        mFrameFieldEngineState = FrameFieldEngineState::CRC15 ;
      }else{
        mFrameFieldEngineState = FrameFieldEngineState::CRC17 ;
        mUnstuffingActive = false ;
      }
      mFieldBitIndex = 0 ;
      mCRC15 = mCRC15Accumulator ;
      mCRC17 = mCRC17Accumulator ;
      mCRC21 = mCRC21Accumulator ;
    }
    break ;
  case FrameFieldEngineState::CRC15 :
    enterBitInCRC15 (inBit) ;
    addMark (inSampleNumber, AnalyzerResults::Dot);
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 15) {
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
      addBubble (CRC15_FIELD_RESULT, mCRC15, mCRC15Accumulator, inSampleNumber + samplesPerBit / 2) ;
    }
    break ;
  case FrameFieldEngineState::CRC17 :
    if ((mFieldBitIndex % 5) != 0) {
      enterBitInCRC17 (inBit) ;
      addMark (inSampleNumber, AnalyzerResults::Dot);
    }else if (inBit == mPreviousBit) {
      enterInErrorMode (inSampleNumber) ;
    }else{
      addMark (inSampleNumber, AnalyzerResults::X);
    }
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 22) {
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::CRCDEL ;
      addBubble (CRC17_FIELD_RESULT, mCRC17, mCRC17Accumulator, inSampleNumber + samplesPerBit / 2) ;
    }
    break ;
  case FrameFieldEngineState::CRCDEL :
    mUnstuffingActive = false ;
    if (inBit) {
      addMark (inSampleNumber, AnalyzerResults::One) ;
    }else{
      enterInErrorMode (inSampleNumber) ;
    }
    mStartOfFieldSampleNumber = inSampleNumber + samplesPerBit / 2 ;
    mFrameFieldEngineState = FrameFieldEngineState::ACK ;
    break ;
  case FrameFieldEngineState::ACK :
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 1) { // ACK SLOT
      addMark (inSampleNumber, inBit ? AnalyzerResults::ErrorSquare : AnalyzerResults::Square);
    }else{ // ACK DELIMITER
      addBubble (ACK_FIELD_RESULT, 0, 0, inSampleNumber + samplesPerBit / 2) ;
      if (inBit) {
        addMark (inSampleNumber, AnalyzerResults::One) ;
      }else{
        enterInErrorMode (inSampleNumber) ;
      }
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::ENDOFFRAME ;
    }
    break ;
  case FrameFieldEngineState::ENDOFFRAME :
    if (inBit) {
      addMark (inSampleNumber, AnalyzerResults::One) ;
    }else{
      enterInErrorMode (inSampleNumber) ;
    }
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 7) {
      addBubble (EOF_FIELD_RESULT, 0, 0, inSampleNumber + samplesPerBit / 2) ;
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::INTERMISSION ;
    }
    break ;
  case FrameFieldEngineState::INTERMISSION :
    if (inBit) {
      addMark (inSampleNumber, AnalyzerResults::One) ;
    }else{
      enterInErrorMode (inSampleNumber) ;
    }
    mFieldBitIndex ++ ;
    if (mFieldBitIndex == 3) {
      addBubble (INTERMISSION_FIELD_RESULT, 0, 0, inSampleNumber + samplesPerBit / 2) ;
      mFieldBitIndex = 0 ;
      mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
    }
    break ;
  case FrameFieldEngineState::STUFF_ERROR :
    mUnstuffingActive = false ;
    addMark (inSampleNumber, AnalyzerResults::ErrorDot);
    if (mPreviousBit != inBit) {
      mConsecutiveBitCountOfSamePolarity = 1 ;
      mPreviousBit = inBit ;
    }else if (inBit) {
      mConsecutiveBitCountOfSamePolarity += 1 ;
      if (mConsecutiveBitCountOfSamePolarity == 11) {
        addBubble (CAN_ERROR_RESULT, 0, 0, inSampleNumber + samplesPerBit / 2) ;
        mFrameFieldEngineState = FrameFieldEngineState::IDLE ;
      }
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
  const bool bit20 = (mCRC15Accumulator & (1 << 20)) != 0 ;
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
                                       const U64 inEndSampleNumber) {
  Frame frame ;
  frame.mType = inBubbleType ;
  frame.mFlags = 0 ;
  frame.mData1 = inData1 ;
  frame.mData2 = inData2 ;
  frame.mStartingSampleInclusive = mStartOfFieldSampleNumber ;
  frame.mEndingSampleInclusive = inEndSampleNumber ;
  mResults->AddFrame (frame) ;
  ReportProgress (frame.mEndingSampleInclusive) ;
//--- Prepare for next bubble
  mStartOfFieldSampleNumber = inEndSampleNumber ;
}

//--------------------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzer::enterInErrorMode (const U64 inSampleNumber) {
  addMark (inSampleNumber, AnalyzerResults::ErrorX);
  mStartOfFieldSampleNumber = inSampleNumber ;
  mFrameFieldEngineState = FrameFieldEngineState::STUFF_ERROR ;
  mUnstuffingActive = false ;
}

//--------------------------------------------------------------------------------------------------


