#include "CANFDMolinaroAnalyzerResults.h"
#include <AnalyzerHelpers.h>
#include "CANFDMolinaroAnalyzer.h"
#include "CANFDMolinaroAnalyzerSettings.h"
#include <iostream>
#include <fstream>
#include <sstream>

//----------------------------------------------------------------------------------------

CANFDMolinaroAnalyzerResults::CANFDMolinaroAnalyzerResults (CANFDMolinaroAnalyzer* analyzer,
                                                            CANFDMolinaroAnalyzerSettings* settings ) :
AnalyzerResults(),
mSettings (settings),
mAnalyzer (analyzer) {
}

//----------------------------------------------------------------------------------------

CANFDMolinaroAnalyzerResults::~CANFDMolinaroAnalyzerResults () {
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerResults::GenerateText (const Frame & inFrame,
                                               const DisplayBase inDisplayBase,
                                               const bool inBubbleText,
                                               std::stringstream & ioText) {
  char numberString [128] = "" ;
  switch (inFrame.mType) {
  case STANDARD_IDENTIFIER_FIELD_RESULT :
//    AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 12, numberString, 128);
    snprintf (numberString, 128, "0x%03llX", inFrame.mData1) ;
    ioText << ((inFrame.mData2 == 0) ? "Std Remote idf: " : "Std Data idf: ") ;
    ioText << numberString ;
    ioText << "\n" ;
    break ;
  case EXTENDED_IDENTIFIER_FIELD_RESULT :
//     AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 32, numberString, 128);
    snprintf (numberString, 128, "0x%08llX", inFrame.mData1) ;
    ioText << ((inFrame.mData2 == 0) ? "Ext Remote idf: " : "Ext Data idf: ") ;
    ioText << numberString ;
    ioText << "\n" ;
    break ;
  case CAN20B_CONTROL_FIELD_RESULT :
    if (!inBubbleText) {
      ioText << "  " ;
    }
    ioText << "Ctrl: " << inFrame.mData1 << "\n" ;
    break ;
  case CANFD_CONTROL_FIELD_RESULT :
    if (!inBubbleText) {
      ioText << "  " ;
    }
    ioText << "Ctrl: " << inFrame.mData1 << " (FDF" ;
    if ((inFrame.mData2 & 1) != 0) {
      ioText << ", BRS" ;
    }
    if ((inFrame.mData2 & 2) != 0) {
      ioText << ", ESI" ;
    }
    ioText << ")\n" ;
    break ;
  case DATA_FIELD_RESULT :
    if (!inBubbleText) {
      ioText << "  " ;
    }
//    AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 8, numberString, 128);
    snprintf (numberString, 128, "0x%02llX", inFrame.mData1) ;
    ioText << "D" << inFrame.mData2 << ": " << numberString << "\n" ;
    break ;
  case CRC15_FIELD_RESULT : // Data1: CRC, Data2: is 0 if CRC ok
    if (!inBubbleText) {
      ioText << "  " ;
    }
    snprintf (numberString, 128, "0x%04llX", inFrame.mData1) ;
    ioText << "CRC15: " << numberString ;
    // AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 16, numberString, 128);
    if (inFrame.mData2 != 0) {
      ioText << " (error)" ;
    }
    ioText << "\n" ;
    break ;
  case CRC17_FIELD_RESULT : // Data1: CRC, Data2: is 0 if CRC ok
    if (!inBubbleText) {
      ioText << "  " ;
    }
    // AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 20, numberString, 128);
    snprintf (numberString, 128, "0x%05llX", inFrame.mData1) ;
    ioText << "CRC17: " << numberString ;
    if (inFrame.mData2 != 0) {
      ioText << " (error)" ;
    }
    ioText << "\n" ;
    break ;
  case CRC21_FIELD_RESULT : // Data1: CRC, Data2: is 0 if CRC ok
    if (!inBubbleText) {
      ioText << "  " ;
    }
    // AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 24, numberString, 128);
    snprintf (numberString, 128, "0x%06llX", inFrame.mData1) ;
    ioText << "CRC21: " << numberString ;
    if (inFrame.mData2 != 0) {
      ioText << " (error)" ;
    }
    ioText << "\n" ;
    break ;
  case ACK_FIELD_RESULT :
    if (inBubbleText) {
      if (inFrame.mData1 != 0) {
        ioText << "NAK\n" ;
      }else{
        ioText << "ACK\n" ;
      }
    }
    break ;
  case SBC_FIELD_RESULT :
    { const bool parityError = (inFrame.mData2 & 1) != 0 ;
      const bool stuffBitCountError = (inFrame.mData2 >> 1) != inFrame.mData1 ;
      if (!inBubbleText) {
        ioText << "  " ;
      }
      ioText << "SBC: " << inFrame.mData1 ;
      if (parityError && stuffBitCountError) {
        ioText << " (error " << (inFrame.mData2 >> 1) << ", P)" ;
      }else if (parityError) {
        ioText << " (error P)" ;
      }else if (stuffBitCountError) {
        ioText << " (error " << (inFrame.mData2 >> 1) << ")" ;
      }
      ioText << "\n" ;
    } break ;
  case EOF_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "EOF\n" ;
    }
    break ;
  case INTERMISSION_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "IFS\n" ;
    }
    break ;
  default :
    if (!inBubbleText) {
      ioText << "  " ;
    }
    ioText << "Error\n" ;
    break ;
  }
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerResults::GenerateBubbleText (const U64 inFrameIndex,
                                                     Channel& channel,
                                                     const DisplayBase inDisplayBase) {
  const Frame frame = GetFrame (inFrameIndex) ;
  std::stringstream text ;
  GenerateText (frame, inDisplayBase, true, text) ;
  ClearResultStrings () ;
  AddResultString (text.str().c_str ()) ;
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerResults::GenerateFrameTabularText (const U64 inFrameIndex,
                                                           const DisplayBase inDisplayBase) {
  #ifdef SUPPORTS_PROTOCOL_SEARCH
    const Frame frame = GetFrame (inFrameIndex) ;
    std::stringstream text ;
    GenerateText (frame, inDisplayBase, false, text) ;
    ClearTabularText () ;
    if (text.str().length () > 0) {
      AddTabularText (text.str().c_str ()) ;
    }
  #endif
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerResults::GenerateExportFile (const char* file,
                                                       DisplayBase display_base,
                                                       U32 export_type_user_id) {
  std::ofstream file_stream (file, std::ios::out) ;

  const U64 trigger_sample = mAnalyzer->GetTriggerSample();
  const U32 sample_rate = mAnalyzer->GetSampleRate();

  file_stream << "Time [s],Value" << std::endl;

  U64 num_frames = GetNumFrames();
  for(U32 i = 0 ; i < num_frames ; i++) {
    Frame frame = GetFrame( i );

    char time_str[128] ;
    AnalyzerHelpers::GetTimeString (frame.mStartingSampleInclusive, trigger_sample, sample_rate, time_str, 128);

    char number_str[128] ;
    AnalyzerHelpers::GetNumberString (frame.mData1, display_base, 8, number_str, 128) ;

    file_stream << time_str << "," << number_str << std::endl;

    if (UpdateExportProgressAndCheckForCancel (i, num_frames) == true) {
      file_stream.close () ;
      return ;
    }
  }

  file_stream.close();
}

//----------------------------------------------------------------------------------------


void CANFDMolinaroAnalyzerResults::GeneratePacketTabularText (U64 packet_id, DisplayBase display_base) {
  //not supported
}

//----------------------------------------------------------------------------------------

void CANFDMolinaroAnalyzerResults::GenerateTransactionTabularText (U64 transaction_id,
                                                                   DisplayBase display_base) {
  //not supported
}

//----------------------------------------------------------------------------------------
