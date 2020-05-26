#ifndef CANMOLINARO_ANALYZER_RESULTS
#define CANMOLINARO_ANALYZER_RESULTS

//--------------------------------------------------------------------------------------------------

#include <AnalyzerResults.h>

//--------------------------------------------------------------------------------------------------

enum CanFrameType {
  STANDARD_IDENTIFIER_FIELD_RESULT,
  EXTENDED_IDENTIFIER_FIELD_RESULT,
  CAN20B_CONTROL_FIELD_RESULT,
  CANFD_CONTROL_FIELD_RESULT,
  DATA_FIELD_RESULT,
  CRC15_FIELD_RESULT,
  CRC17_FIELD_RESULT,
  CRC21_FIELD_RESULT,
  ACK_FIELD_RESULT,
  EOF_FIELD_RESULT,
  INTERMISSION_FIELD_RESULT,
  CAN_ERROR_RESULT
} ;

//--------------------------------------------------------------------------------------------------

class CANFDMolinaroAnalyzer;
class CANFDMolinaroAnalyzerSettings;

//--------------------------------------------------------------------------------------------------

class CANFDMolinaroAnalyzerResults : public AnalyzerResults {
public:
	CANFDMolinaroAnalyzerResults( CANFDMolinaroAnalyzer* analyzer, CANFDMolinaroAnalyzerSettings* settings );
	virtual ~CANFDMolinaroAnalyzerResults();

	virtual void GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base );
	virtual void GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id );

	virtual void GenerateFrameTabularText(U64 frame_index, DisplayBase display_base );
	virtual void GeneratePacketTabularText( U64 packet_id, DisplayBase display_base );
	virtual void GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base );

protected: //functions
  void GenerateText (const Frame & inFrame,
                     const DisplayBase inDisplayBase,
                     const bool inBubbleText,
                     std::stringstream & ioText) ;

protected:  //vars
	CANFDMolinaroAnalyzerSettings* mSettings;
	CANFDMolinaroAnalyzer* mAnalyzer;
};

//--------------------------------------------------------------------------------------------------

#endif //CANMOLINARO_ANALYZER_RESULTS
